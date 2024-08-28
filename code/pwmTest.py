import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18, 1000)  
pwm.start(0)  

try:
    while True:
        duty_cycle = input("Enter duty cycle (0-100) or press Enter to stop: ")
        
        if duty_cycle == "":
            print("Stopping PWM...")
            pwm.ChangeDutyCycle(0)
            break

        try:
            duty_cycle = float(duty_cycle)
            if 0 <= duty_cycle <= 100:
                pwm.ChangeDutyCycle(duty_cycle)
                print(f"Outputting at {duty_cycle}% duty cycle...")
            else:
                print("Please enter a value between 0 and 100.")
        except ValueError:
            print("Invalid input. Please enter a numeric value between 0 and 100.")

finally:
    pwm.stop()
    GPIO.cleanup()

print("PWM output stopped and GPIO cleaned up.")
#3 - 1V
#15 - 2V
#26 - 3v
