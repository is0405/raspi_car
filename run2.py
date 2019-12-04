import time
import RPi.GPIO as GPIO
import go

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

def forward( cycle_l, cycle_r ):    
    pwmL.ChangeDutyCycle(cycle_l)
    GPIO.output(17, 0)
    GPIO.output(27, 1)
    pwmR.ChangeDutyCycle(cycle_r)
    GPIO.output(9, 1)
    GPIO.output(10, 0)

start_x = 0.0
final = False
result = go.quinic(start_x)
while( not result[0] ):
    forward(100, 100)
    time.sleep(1)
    forward(0, 0)
    if start_x >= 3.6:
        final = True
        break
    start_x += 0.7
    result = go.quinic(start_x)
    if result[3]:
        forward(0,0)
        print("emergency stop")

if final:
    print("Green is not found")
else:
    v_l = result[1] * 100/0.7
    v_r = result[2] * 100/0.7
    
    for i in range(len(v_l)):
        forward(round(v_l[i]), round(v_r[i]))
        time.sleep(0.05)
        
pwmL.stop()
pwmR.stop()

GPIO.cleanup()
time.sleep(1)
