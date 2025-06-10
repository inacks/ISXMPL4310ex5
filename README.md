# IS4310 Python Code Example for Modbus over I2C on Raspberry Pi

This Python script communicates with the IS4310 Modbus RTU chip via I²C using a Raspberry Pi.

It demonstrates:
- How to read a push button (simulating a sensor) and store its state in Holding Register 0.
- How to control an RGB LED (simulating an actuator) using PWM on GPIO pins 12, 13, and 19,  
  based on values in Holding Registers 1, 2, and 3.

A value of 0 turns off the LEDs, and a value of 100 sets them to maximum brightness.

💡 Test this example using the **Kappa4310Rasp Evaluation Board**:  
👉 [https://www.inacks.com/kappa4310rasp](https://www.inacks.com/kappa4310rasp)

📄 Download the **IS4310 datasheet**:  
👉 [https://www.inacks.com/is4310](https://www.inacks.com/is4310)

For more information visit [www.inacks.com](https://www.inacks.com/is4310)
