import RPi.GPIO as GPIO
from time import sleep

servo_pin = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)

pwm=GPIO.PWM(servo_pin, 50)
pwm.start(0)

left = 2.5
neutral = 7.5
right = 12.5

print("begin test")

print("duty cycle", left,"% at left -90 deg")
pwm.ChangeDutyCycle(left)
sleep(1)

print("duty cycle", neutral,"% at 0 deg")
pwm.ChangeDutyCycle(neutral)
#sleep(1)

print("duty cycle",right, "% at right +90 deg")
pwm.ChangeDutyCycle(right)
sleep(1)

print("end of test")

pwm.stop()
GPIO.cleanup()
