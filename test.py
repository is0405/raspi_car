import time
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
    #cv2.line(frame, (0,int(height/2)),(width,int(height/2)),(0,0,255),2)
    cv2.line(frame, (int(width/8),0),(int(width/8),height),(0,0,255),2)
    cv2.line(frame, (int(width/4),0),(int(width/4),height),(0,0,255),2)
    cv2.line(frame, (int(3*width/8),0),(int(3*width/8),height),(0,0,255),2)
    cv2.line(frame, (int(width/2),0),(int(width/2),height),(0,0,255),2)
    cv2.line(frame, (int(5*width/8),0),(int(5*width/8),height),(0,0,255),2)
    cv2.line(frame, (int(3*width/4),0),(int(3*width/4),height),(0,0,255),2)
    cv2.line(frame, (int(7*width/8),0),(int(7*width/8),height),(0,0,255),2)
    

    return frame, mask, x-width/2, y-height/2, ok

def image(img):
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

    return opening
    
def forward( cycle_l, cycle_r ):
    print()
    #pwmL.ChangeDutyCycle(cycle_l)
    #GPIO.output(17, 0)
    #GPIO.output(27, 1)
    #pwmR.ChangeDutyCycle(cycle_r)
    #GPIO.output(9, 1)
    #GPIO.output(10, 0)
    #time.sleep(0.1)

def move( x, y , width):
    if x < -width/2 + width/8:
        forward(75, 90)
        print(1)
    elif x < -width/4:
        forward(80, 90)
        print(2)
    elif x < -width/8:
        forward(80, 85)
        print(3)
    elif width/2 - width/8 < x:
        forward(90, 75)
        print(8)
    elif width/4 < x:
        forward(90, 80)
        print(7)
    elif width/8 < x:
        forward(85, 80)
        print(6)
    else:
        forward(80, 80)
        print(45)

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    # width
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # height
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    while cap.isOpened():
        # get a frame
        _, frame = cap.read()
    
        # BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower = np.array([60/2, 58, 80])
        upper = np.array([170/2, 255, 200])
    
        mask = cv2.inRange(hsv, lower, upper)
        cv2.imshow("mask1",mask)
        mask = image(mask)

        frame, mask, ans_x, ans_y, ok = image_draw(frame, mask, W, H)

        px_count = np.count_nonzero(mask)

        if px_count < 2000:
            ok = False
    
        if px_count > 70000:
            forward(0, 0)
            print("stop")
            break

        if ok:
            move(ans_x, ans_y, W)
        else:
            forward(90, 90)
            print("not found")

        cv2.imshow("frame",frame)
        cv2.imshow("mask2",mask)
        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
