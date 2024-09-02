import RPi.GPIO as GPIO
from time import sleep

servo_pin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)


pwm=GPIO.PWM(servo_pin, 50)
pwm.start(0)

right = 4
neutral = 7.5
left = 12.5

right_30 = 6.0
left_30 = 9

right_60 = 5.3
left_60 = 10

right_80 = 4.5
left_80 = 11

print("begin test")


print("duty cycle",right, "% at right +60 deg")
pwm.ChangeDutyCycle(left_60)
sleep(1)

print("duty cycle",right, "% at right -60 deg")
pwm.ChangeDutyCycle(right_60)
sleep(1)
print("duty cycle",right, "% at right +60 deg")
pwm.ChangeDutyCycle(left_60)
sleep(1)

print("duty cycle",right, "% at right -60 deg")
pwm.ChangeDutyCycle(right_60)
sleep(1)

print("duty cycle",right, "% at right +60 deg")
pwm.ChangeDutyCycle(neutral)
sleep(1)

print("end of test")

pwm.stop()
GPIO.cleanup()

#30 - 4.5,9.5
#54 - 5.5, 10.5
#72 - 6.5, 11.5
