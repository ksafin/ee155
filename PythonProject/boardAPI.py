import spiAPI
import spi_util as su
import motorAPI
import servoAPI
# import stepperAPI


class MotorHat:
    def __init__(self, bus, device):
        self.spi = spiAPI.SpiDev(bus,device)
        self.spi.enableDebug()
        self.enabled = False

        self.motor1 = motorAPI.Motor(self.spi,1)
        self.motor2 = motorAPI.Motor(self.spi,2)
        self.motor3 = motorAPI.Motor(self.spi,3)
        self.motor4 = motorAPI.Motor(self.spi,4)

        self.servo1 = servoAPI.Servo(self.spi, 1)
        self.servo2 = servoAPI.Servo(self.spi, 2)
        self.servo3 = servoAPI.Servo(self.spi, 3)
        self.servo4 = servoAPI.Servo(self.spi, 4)

    def begin(self):
        if self.spi.ping():
            self.spi.initializeRegisters()
            self.enabled = True
        else:
            return false

    def getmotor(self, motornum):
        if not self.enabled:
            return None

        if motornum == 1:
            return self.motor1
        elif motornum == 2:
            return self.motor2
        elif motornum == 3:
            return self.motor3
        elif motornum == 4:
            return self.motor4
        else:
            return None

    def getservo(self, servonum, angle):
        if not self.enabled:
            return None

        if servonum == 1:
            self.servo1.defineangle(angle)
            return self.servo1
        elif servonum == 2:
            self.servo2.defineangle(angle)
            return self.servo2
        elif servonum == 3:
            self.servo3.defineangle(angle)
            return self.servo3
        elif servonum == 4:
            self.servo4.defineangle(angle)
            return self.servo4
        else:
            return None

        