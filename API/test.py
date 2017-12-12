import boardAPI as brd
import time

test = brd.MotorHat(0,0)
worked = test.begin()
print worked

motor1 = test.getmotor(1)
motor1.setPWM(100)
motor1.setPPR(motor1.NEVEREST_60_PPR);
time.sleep(1);
spd = motor1.getSpeed()
curr = motor1.getCurrent()
enc = motor1.getEncoder()
print enc
print curr
print spd
motor1.resetEncoder()
enc = motor1.getEncoder()
print enc
motor1.enableCoast()
motor1.enableBrake()
motor1.setCoastSpeed(1000)
#motor1.release()
fault = motor1.getFault()
print fault
motor1.disablePID()
motor1.stop()
#motor1.setCurrentLimit(10000);
#motor1.setPID(0.8, 0.2, 0.05)