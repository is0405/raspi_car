import time
import RPi.GPIO as GPIO
import cv2
import numpy as np


def image_draw(frame, mask, width, height):
    # calc 
    mu = cv2.moments(mask, False)
    ok = False

    if mu["m00"] != 0:
        x, y = int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
        ok = True
    else :
        x, y = -1, -1

    cv2.circle(frame, (x,y), 4, 255, 2, 4)
    cv2.line(frame, (0,int(height/2)),(width,int(height/2)),(0,0,255),2)
    cv2.line(frame, (int(width/2),0),(int(width/2),height),(0,0,255),2)

    return frame, mask, x-width/2, y-height/2, ok

def image(img):
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

    return opening

def forward( pwmL, pwmR, cycle_l, cycle_r ):
    pwmL.ChangeDutyCycle(cycle_l)
    GPIO.output(17, 0)
    GPIO.output(27, 1)
    pwmR.ChangeDutyCycle(cycle_r)
    GPIO.output(9, 1)
    GPIO.output(10, 0)

def back( pwmL, pwmR, cycle_l, cycle_r ):
    pwmL.ChangeDutyCycle(cycle_l)
    GPIO.output(17, 1)
    GPIO.output(27, 0)
    pwmR.ChangeDutyCycle(cycle_r)
    GPIO.output(9, 0)
    GPIO.output(10, 1)

def move( x, y, width, mode):
    if x < -width/4:
        forward(pwmL, pwmR, 25, 0)
        print(1)
    elif width/4 < x:
        forward(pwmL, pwmR, 0, 25)
        print(8)
    else:
        if mode:
            forward(pwmL, pwmR, 80, 80)
            print(4)
        else:
            forward(pwmL, pwmR, 100, 100)
            print(5)


if __name__ == '__main__':

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(22, GPIO.OUT)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(27, GPIO.OUT)
    GPIO.setup(9, GPIO.OUT)
    GPIO.setup(10, GPIO.OUT)
    GPIO.setup(0, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)

    pwmL = GPIO.PWM(0, 1000)
    pwmR = GPIO.PWM(22, 1000)
    pwmL.start(0)
    pwmR.start(0)
    GPIO.output(24, 1)

    cap = cv2.VideoCapture(0)
    # width
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # height
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    before_ok = False
    count = 0

    while cap.isOpened():
        # get a frame
        _, frame = cap.read()
    
        # BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower = np.array([60/2, 58, 80])
        upper = np.array([170/2, 255, 200])
    
        mask = cv2.inRange(hsv, lower, upper)
        mask = image(mask)

        frame, mask, ans_x, ans_y, ok = image_draw(frame, mask, W, H)

        px_count = np.count_nonzero(mask)

        if px_count < 100:
           ok = False

        if ok:
            before_ok = True

        save_mode = False
        if px_count > 50000:
            forward(pwmL, pwmR, 0, 0)
            break
        elif px_count > 30000:
            save_mode = True

        if ok:
            move(ans_x, ans_y, W, save_mode)
            count = 0
            cv2.imwrite("frame" + str(px_count) + ".png", frame)
            cv2.imwrite("mask" + str(px_count) + ".png", mask)
            print("found " + str(ans_x) +" " + str(ans_y))
        else:
            if before_ok:
                count += 1
                if count > 50:
                    before_ok = False
                    forward(pwmL, pwmR, 28, 0)
                    time.sleep(1)
                    forward(pwmL, pwmR, 0, 28)
                    print("round")
                else:
                    back(pwmL, pwmR, 30, 30)
                    print("back")
            else:
                forward(pwmL, pwmR, 80, 80)
                print("not found")


    cap.release()
    cv2.destroyAllWindows()

    pwmL.stop()
    pwmR.stop()

    GPIO.cleanup()
    time.sleep(1)
