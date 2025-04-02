# **Sentry Turret Project**  

This project is a **camera-based sentry turret** that tracks and fires at targets using a **STM32 microcontroller, a laptop, and an Arduino**.  

## **How It Works:**  
- 📷 **STM32** captures camera frames using the OV7670 module, processes images using DCMI and transmits them to the laptop via DMA.  
- 💻 **Laptop** processes the frames and sends aiming commands to the Arduino UNO via UART.  
- 🎯 **Arduino** controls the **servo motors** to swivel and aim, along with manual joystick control and a safety firing pin.  
- 🔫 Fires a **rubber band** at the target when aligned.  

This system combines **real-time image transmission, computer vision, and motor control**  

Check out these demos!

[Demo #1](https://youtube.com/shorts/_wEvguuBnUk?feature=share)

[Demo #2](https://youtube.com/shorts/TP8RxhQHSZo?feature=share)

