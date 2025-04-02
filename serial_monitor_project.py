import time

import click
import cv2 as cv
import numpy as np
import face_recognition as face_recognition
from serial import Serial

PORT = "COM3"
BAUDRATE = 921600  # 921600

PREAMBLE = "!START!\r\n"
DELTA_PREAMBLE = "!DELTA!\r\n"
SUFFIX = "!END!\r\n"

ROWS = 144
COLS = 174


@click.command()
@click.option(
    "-p", "--port", default=PORT, help="Serial (COM) port of the target board"
)
@click.option("-br", "--baudrate", default=BAUDRATE, help="Serial port baudrate")
@click.option("--timeout", default=1, help="Serial port timeout")
@click.option("--rows", default=ROWS, help="Number of rows in the image")
@click.option("--cols", default=COLS, help="Number of columns in the image")
@click.option("--preamble", default=PREAMBLE, help="Preamble string before the frame")
@click.option(
    "--delta_preamble",
    default=DELTA_PREAMBLE,
    help="Preamble before a delta update during video compression.",
)
@click.option(
    "--suffix", default=SUFFIX, help="Suffix string after receiving the frame"
)
@click.option(
    "--short-input",
    is_flag=True,
    default=False,
    help="If true, input is a stream of 4b values",
)
@click.option("--rle", is_flag=True, default=False, help="Run-Length Encoding")
@click.option("--quiet", is_flag=True, default=False, help="Print fewer messages")
def monitor(
    port: str,
    baudrate: int,
    timeout: int,
    rows: int,
    cols: int,
    preamble: str,
    delta_preamble: str,
    suffix: str,
    short_input: bool,
    rle: bool,
    quiet: bool,
) -> None:
    """
    Display images transferred through serial port. Press 'q' to close.
    """
    prev_frame_ts = None  # keep track of frames per second
    frame = None

    click.echo(
        f"Opening communication on port {port} with baudrate {baudrate}")

    if isinstance(suffix, str):
        suffix = suffix.encode("ascii")

    if isinstance(preamble, str):
        preamble = preamble.encode("ascii")

    if isinstance(delta_preamble, str):
        delta_preamble = delta_preamble.encode("ascii")

    img_rx_size = rows * cols
    if short_input:
        img_rx_size //= 2
    if rle:
        img_rx_size *= 2

    partial_frame_counter = 0  # count partial updates every full frame

    # setup connection with the arduino
    arduino = Serial(port='COM4', baudrate=9600, timeout=1)
    time.sleep(2)  # Wait for connection

    # keep track of arduino turret's vertical angle (70=highest, 100=lowest)
    verticalState = 90

    with Serial(port, baudrate, timeout=timeout) as ser:
        while True:
            # (commented out to de-clutter)
            # if not quiet:
            #     click.echo("Waiting for input data...")

            full_update = wait_for_preamble(ser, preamble, delta_preamble)

            if full_update:
                # (commented out to de-clutter)
                # click.echo(
                #     f"Full update (after {partial_frame_counter} partial updates)"
                # )
                partial_frame_counter = 0
            else:
                if not quiet:
                    click.echo("Partial update")
                partial_frame_counter += 1

                if frame is None:
                    click.echo(
                        "No full frame has been received yet. Skipping partial update."
                    )
                    continue

            # (commented out to de-clutter)
            # if not quiet:
            #     click.echo("Receiving picture...")

            try:
                raw_data = get_raw_data(ser, img_rx_size, suffix)
                # (commented out to de-clutter)
                # if not quiet:
                #     click.echo(f"Received {len(raw_data)} bytes")
            except ValueError as e:
                click.echo(f"Error while waiting for frame data: {e}")

            if short_input:
                raw_data = (
                    expand_4b_to_8b(raw_data)
                    if not rle
                    else expand_4b_to_8b_rle(raw_data)
                )
            elif rle and len(raw_data) % 2 != 0:
                # sometimes there serial port picks up leading 0s
                # discard these
                raw_data = raw_data[1:]

            if rle:
                raw_data = decode_rle(raw_data)

            try:
                new_frame = load_raw_frame(raw_data, rows, cols)
            except ValueError as e:
                click.echo(f"Malformed frame. {e}")
                continue

            frame = new_frame if full_update else frame + new_frame

            # calculate fps
            # (commented out to de-clutter)
            # now = time.time()
            # if prev_frame_ts is not None:
            #     try:
            #         fps = 1 / (now - prev_frame_ts)
            #         click.echo(f"Frames per second: {fps:.2f}")
            #     except ZeroDivisionError:
            #         click.echo("FPS too fast to measure")
            # prev_frame_ts = now

            # Sharpening the image
            sharpening_kernel = np.array([[0, -1, 0],
                                          [-1, 5, -1],
                                          [0, -1, 0]])
            frame = cv.filter2D(frame, -1, sharpening_kernel)

            # face recognition
            face_locations = face_recognition.face_locations(frame)

            # make the rectangle green
            frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)

            # Only process if faces are detected
            if face_locations:
                # Calculate areas of all detected faces
                face_areas = [
                    (right - left) * (bottom - top) 
                    for (top, right, bottom, left) in face_locations
                ]
                
                # Find index of largest face
                largest_face_idx = face_areas.index(max(face_areas))
                top, right, bottom, left = face_locations[largest_face_idx]

                # Draw rectangle around largest face
                cv.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

                # Calculate face center coordinates
                face_center_x = (left + right) // 2
                face_center_y = (top + bottom) // 2

                # Get image center coordinates
                image_center_x = frame.shape[1] // 2
                image_center_y = frame.shape[0] // 2

                # Calculate position offsets
                dx = face_center_x - image_center_x
                dy = face_center_y - map_value(verticalState)

                # Draw visual markers
                cv.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)
                cv.line(frame, (image_center_x, image_center_y),
                        (face_center_x, face_center_y), (255, 0, 0), 2)
                cv.putText(frame, f"dx:{dx}, dy:{dy}", (face_center_x + 10, face_center_y),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Send commands to Arduino based on largest face
                if dx < -10:
                    send_command(arduino=arduino, command="left")
                elif dx > 10:
                    send_command(arduino=arduino, command="right")
                elif dy < 5:
                    verticalState = send_command(arduino=arduino, command="up")
                elif dy > 30:
                    verticalState = send_command(arduino=arduino, command="down")
                else:
                    send_command(arduino=arduino, command="fire")

            # display the frame
            # cv.namedWindow("Video Stream", cv.WINDOW_KEEPRATIO)
            # cv.imshow("Video Stream", frame)
            # Calculate rotated dimensions
            rows_rotated = cols * 3  # scale to make bigger
            cols_rotated = rows * 3

            # Update window size to match rotated image
            cv.namedWindow("Video Stream", cv.WINDOW_NORMAL)
            cv.resizeWindow("Video Stream", cols_rotated, rows_rotated)
            cv.imshow("Video Stream", frame)

            # Wait for 'q' to stop the program
            if cv.waitKey(1) == ord("q"):
                break

    cv.destroyAllWindows()


def wait_for_preamble(ser: Serial, preamble: str, partial_preamble: str) -> bool:
    """
    Wait for a preamble string in the serial port.

    Returns `True` if next frame is full, `False` if it's a partial update.
    """
    while True:
        try:
            line = ser.readline()
            if line == preamble:
                return True
            elif line == partial_preamble:
                return False
        except UnicodeDecodeError:
            pass


def get_raw_data(ser: Serial, num_bytes: int, suffix: bytes = b"") -> bytes:
    """
    Get raw frame data from the serial port.
    """
    rx_max_len = num_bytes + len(suffix)
    max_tries = 10_000
    raw_img = b""

    for _ in range(max_tries):
        raw_img += ser.read(max(1, ser.in_waiting))

        suffix_idx = raw_img.find(suffix)
        if suffix_idx != -1:
            raw_img = raw_img[:suffix_idx]
            break

        if len(raw_img) >= rx_max_len:
            raw_img = raw_img[:num_bytes]
            break
    else:
        raise ValueError("Max tries exceeded.")

    return raw_img


def expand_4b_to_8b(raw_data: bytes) -> bytes:
    """
    Transform an input of 4-bit encoded values into a string of 8-bit values.

    For example, value 0xFA gets converted to [0xF0, 0xA0]
    """
    return bytes(val for pix in raw_data for val in [pix & 0xF0, (pix & 0x0F) << 4])


def expand_4b_to_8b_rle(raw_data: bytes) -> bytes:
    """
    Transform an input of 4-bit encoded RLE values into a string of 8-bit values.

    For example, value 0xFA gets converted to [0xF0, 0x0A]
    """
    return bytes(val for pix in raw_data for val in [pix & 0xF0, pix & 0x0F])


def decode_rle(raw_data: bytes) -> bytes:
    """
    Decode Run-Length Encoded data.
    """
    return bytes(
        val
        for pix, count in zip(raw_data[::2], raw_data[1::2])
        for val in [pix] * count
    )


# def load_raw_frame(raw_data: bytes, rows: int, cols: int) -> np.array:
#     return np.frombuffer(raw_data, dtype=np.uint8).reshape((rows, cols, 1))


# # rotated version because of my camera setup
def load_raw_frame(raw_data: bytes, rows: int, cols: int) -> np.array:
    frame = np.frombuffer(raw_data, dtype=np.uint8).reshape((rows, cols, 1))
    return cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)


def send_command(arduino, command) -> int:
    """
    Send a command to the arduino over UART and get its response.
    If the response is a VerticalState, return as int. else return -1
    """
    # Add newline terminator and send full command
    arduino.write(f"{command}\n".encode())  # Send command with newline
    time.sleep(0.1)  # Short delay
    response = arduino.readline().decode().strip()
    click.echo(f"Arduino says: {response}")

    # Check if response contains "VerticalState"
    if "VerticalState:" in response:
        try:
            # Extract and convert value
            return int(response.split("VerticalState:")[-1].strip())
        except ValueError:
            pass  # If conversion fails, return -1

    return -1  # Default if no vertical state is found


def map_value(x, in_min=70, in_max=100, out_min=0, out_max=174):
    """
    - Used for vertical aiming since the camera doesn't move up and down
    - Maps 70-100 from verticalState to 0-174 in monitor
    """
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min


if __name__ == "__main__":
    monitor()
