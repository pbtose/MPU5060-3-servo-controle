from imu import MPU6050  # Make sure this is your MPU6050 class
from time import sleep
from machine import Pin, I2C, PWM

# Setup I2C on Raspberry Pi Pico W
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # Adjust the SCL and SDA pins if necessary
imu = MPU6050(i2c)  # MPU6050 address

# Setup PWM for the first servo motor
servo1_pin = Pin(4, Pin.OUT)
pwm1 = PWM(servo1_pin)
pwm1.freq(50)



# Setup PWM for the second servo motor
servo2_pin = Pin(5, Pin.OUT)
pwm2 = PWM(servo2_pin)
pwm2.freq(50)

# Setup PWM for the third servo motor
servo3_pin = Pin(6, Pin.OUT)
pwm3 = PWM(servo3_pin)
pwm3.freq(50)

# Function to set the angle of a specific servo
def set_servo_angle(pwm, Angle):
    duty = Angle / 18 + 2
    pwm.duty_u16(int(duty * 65535 / 100))  # Convert duty cycle to 16-bit value
    sleep(0.0)  # Allow some time for the servo to move

while True:
    # Read Accelerometer raw values
    ay = round(imu.accel.y, 2)
    ax = round(imu.accel.x, 2)

    # Convert accelerometer X,Y axis values from -2 to 2 g
    in_min = -2
    in_max = 2
    out_min = 0
    out_max = 180

    # Compute the servo angles based on Ay and Ax
    angle1 = int((ay - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    angle2 = int((ay - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    angle3 = int((ax - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    # Limit angles to valid range (0 to 180 degrees)
    angle1 = max(0, min(180, angle1))
    angle2 = max(0, min(180, angle2))
    angle3 = max(0, min(180, angle3))

    # Move both servos
    set_servo_angle(pwm1, angle1)  # Rotate the first servo
    set_servo_angle(pwm2, angle2)  # Rotate the second servo
    set_servo_angle(pwm3, angle3)  # Rotate the third servo

    sleep(0.08)

