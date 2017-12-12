import boardAPI as brd
import time

test = brd.MotorHat(0,0)
worked = test.begin()
print worked

motor1 = test.getmotor(1)

motor1.setPWM(0);

while True:
    print motor1.getEncoder()
    time.sleep(0.001)
