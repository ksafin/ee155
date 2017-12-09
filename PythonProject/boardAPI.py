import spidev
import time

import spi_util as su
import motorAPI
import servoAPI
# import stepperAPI


def MotorHat:

    def __init__(self, bus, device):
        self.spi_obj = spiInit(bus,device)

        self.motor1 = motorAPI.Motor(self.spi_obj,1)
        self.motor2 = motorAPI.Motor(self.spi_obj,2)
        self.motor3 = motorAPI.Motor(self.spi_obj,3)
        self.motor4 = motorAPI.Motor(self.spi_obj,4)

    def Begin(self):
