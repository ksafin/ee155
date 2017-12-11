import boardAPI as brd
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
test_brd = brd.MotorHat(0,0)

cnt = 0
while(True):
    cnt += 1
    print("Cycle " + str(cnt))
    
    time.sleep(.2)