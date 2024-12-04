import RPi.GPIO as GPIO
import time
import cv2


TRIG_PIN = 23
ECHO_PIN = 24  
LED_PIN = 2    

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)


def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance


def detect_motion(frame1, frame2):
    diff = cv2.absdiff(frame1, frame2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(thresh, None, iterations=3)
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        (x, y, w, h) = cv2.boundingRect(contour)
        if cv2.contourArea(contour) < 700:
            continue

        return True

    return False



try:
    while True:
        distance = get_distance()
        print("Distance:", distance, "cm")

        if distance < 30:
            GPIO.output(LED_PIN, True)
            print("Motion detected")

            cap = cv2.VideoCapture(0)
            ret, frame1 = cap.read()
            ret, frame2 = cap.read()

            start_time = time.time()
            show_video = True

            while show_video:
                distance = get_distance()

                ret, frame3 = cap.read()

                if detect_motion(frame1, frame3):
                     print("Human Motion Detected")
                     start_time = time.time()

                if time.time() - start_time >= 0.5:
                   show_video = False


                cv2.imshow("Video", frame1)


                frame1 = frame2
                frame2 = frame3

                if cv2.waitKey(1) == 27:
                    show_video = False


            cap.release()
            cv2.destroyAllWindows()
            GPIO.output(LED_PIN, False)

        time.sleep(0.1)


except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
    cv2.destroyAllWindows()