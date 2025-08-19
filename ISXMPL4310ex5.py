# IS4310 Modbus Code Example for Raspberry Pi
# ----------------------------------------------------------
# This Python script communicates with the IS4310 Modbus RTU chip via I2C using a Raspberry Pi.
# It demonstrates how to read a push button (simulating a sensor) and store its value in Holding Register 0.
# It also controls an RGB LED (simulating an actuator) using PWM pins 12, 13, and 19, based on the values in Holding Registers 1, 2, and 3.
# A value of 0 turns off the LEDs, and a value of 100 sets them to maximum brightness.
#
# You can test this code using the **Kappa4310Rasp Evaluation Board**.
# Buy it at: **www.inacks.com/kappa4310rasp**
#
# Download the **IS4310 datasheet** at:
# https://www.inacks.com/is4310


from smbus2 import SMBus, i2c_msg
import RPi.GPIO as GPIO
import time

I2C_BUS = 1  # I2C bus number on Raspberry Pi (usually 1)
DEVICE_ADDRESS = 0x11  # 7-bit I2C address of the IS4310 Modbus RTU chip
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering scheme

# Define GPIO pins for three LEDs and push button
led_pin1 = 12
led_pin2 = 13
led_pin3 = 19
push_button_pin = 26

# Setup push button pin as input with internal pull-down resistor enabled
GPIO.setup(push_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Setup LED pins as outputs
GPIO.setup(led_pin1, GPIO.OUT)
GPIO.setup(led_pin2, GPIO.OUT)
GPIO.setup(led_pin3, GPIO.OUT)

# Initialize PWM on LED pins at 1 kHz frequency
pwm1 = GPIO.PWM(led_pin1, 1000)
pwm2 = GPIO.PWM(led_pin2, 1000)
pwm3 = GPIO.PWM(led_pin3, 1000)

# Start PWM with 0% duty cycle (LEDs off initially)
pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

def write_register(register, data):
    """
    Write a 16-bit data value to a 16-bit register address on the I2C device.
    
    :param register: 16-bit register address (split into high and low bytes)
    :param data: 16-bit data to write (split into high and low bytes)
    """
    high_addr = (register >> 8) & 0xFF  # Extract high byte of register address
    low_addr = register & 0xFF          # Extract low byte of register address
    data_high = (data >> 8) & 0xFF      # Extract high byte of data
    data_low = data & 0xFF              # Extract low byte of data

    # Open I2C bus, send write message: [register high, register low, data high, data low]
    with SMBus(I2C_BUS) as bus:
        msg = i2c_msg.write(DEVICE_ADDRESS, [high_addr, low_addr, data_high, data_low])
        bus.i2c_rdwr(msg)

def read_register(start_register):
    """
    Read a 16-bit value from a 16-bit register address on the I2C device.
    
    :param start_register: 16-bit register address to read from
    :return: 16-bit integer value read (big-endian)
    """
    high_addr = (start_register >> 8) & 0xFF  # High byte of register address
    low_addr = start_register & 0xFF          # Low byte of register address

    with SMBus(I2C_BUS) as bus:
        # Write register address first to set internal pointer
        write_msg = i2c_msg.write(DEVICE_ADDRESS, [high_addr, low_addr])
        # Prepare to read 2 bytes from the device
        read_msg = i2c_msg.read(DEVICE_ADDRESS, 2)
        bus.i2c_rdwr(write_msg, read_msg)

        data = list(read_msg)  # Read bytes as list of ints
        # Combine high and low bytes into 16-bit integer (big-endian)
        value = (data[0] << 8) | data[1]
        return value

try:
    while True:
        # Read push button state (0 or 1)
        button_value = GPIO.input(push_button_pin)

        # Write button state to register 0 of the device
        write_register(0, button_value)

        # Read PWM values from registers 1, 2, and 3
        pwm_val1 = read_register(1)
        pwm_val2 = read_register(2)
        pwm_val3 = read_register(3)

        # Cap PWM values at max 100 to avoid invalid duty cycles
        if pwm_val1 > 100:
            pwm_val1 = 100
        if pwm_val2 > 100:
            pwm_val2 = 100
        if pwm_val3 > 100:
            pwm_val3 = 100

        # Calculate duty cycles by inverting the PWM value (100 - value)
        # abs() used to ensure positive duty cycle, just in case
        duty1 = abs(pwm_val1 - 100)
        duty2 = abs(pwm_val2 - 100)
        duty3 = abs(pwm_val3 - 100)

        # Print duty cycle values for debugging (tab-separated)
        print(f"{duty1}\t{duty2}\t{duty3}")

        # Update PWM duty cycles to control LED brightness
        pwm1.ChangeDutyCycle(duty1)
        pwm2.ChangeDutyCycle(duty2)
        pwm3.ChangeDutyCycle(duty3)

        # Small delay to avoid excessive CPU load
        time.sleep(0.05)

except KeyboardInterrupt:
    # Gracefully handle Ctrl+C exit
    print("Exiting...")

finally:
    # Stop all PWM signals and cleanup GPIO pins on exit
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    GPIO.cleanup()

