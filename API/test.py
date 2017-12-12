import boardAPI as brd
import time

test = brd.MotorHat(0,0)
worked = test.begin()
print worked

motor1 = test.getmotor(1)

motor1.setPPR(420.0)
motor1.rotatePWM(200,10)

