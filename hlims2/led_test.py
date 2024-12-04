import RPi.GPIO as GPIO
import time

LED_PIN = 2

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(LED_PIN, True)
        print("LED ON")
        time.sleep(0.5)
        GPIO.output(LED_PIN, False)
        print("LED OFF")
        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()