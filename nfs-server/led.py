import RPi.GPIO as GPIO
import time

LED_RED_GPIO = 13
LED_GREEN_GPIO = 26 
LED_BLUE_GPIO = 19

try:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_RED_GPIO, GPIO.OUT)
    GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)
    GPIO.setup(LED_BLUE_GPIO, GPIO.OUT)

    GPIO.output(LED_RED_GPIO, False)
    GPIO.output(LED_GREEN_GPIO, False)
    GPIO.output(LED_BLUE_GPIO, False)

    #pwm1 = GPIO.PWM(LED_BLUE_GPIO, 100) 
    #pwm1.start(0)

    while True:
        print(".")
        GPIO.output(LED_RED_GPIO, 1)
        #GPIO.output(LED_GREEN_GPIO, 1)
        #GPIO.output(LED_BLUE_GPIO, 1)
        #time.sleep(0.5)
        #GPIO.output(LED_BLUE_GPIO, 0)
        #time.sleep(0.5)
        #pwm1.ChangeDutyCycle(100)
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
    
