import packet_util as pu
import spi_util as su
import spiAPI

class Servo:
    def __init__(self,spi_obj,servo_id):
        self.spi = spi_obj
        self.id = servo_id
        self.angle = 0

    def defineangle(self, angle):
        self.angle = angle

    # Set Servo Angle, from 0 to defined angle
    # angle -- a value from 0 to the servos defined angle, to which to advance
    # Function ID: 0x11
    def setangle(self, angle):
        # Do not execute if angle is out of range or servo is continuous
        if (angle > self.angle) or (angle < 0) or (self.angle == 0):
            return False

        # Convert to proper angle range (0 to 180)
        newangle = int(round(angle * (180/self.angle)))

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x11, self.id, list([newangle]))
        self.spi.writeBytes(packet)
        return True

    # Set Speed for continuous servo
    # Speed -- a value from -90 to 90, indicating both direction and speed
    # Function ID: 0x12
    def setspeed(self, speed):
        # Do not execute if speed is out of range or servo is not continuous
        if (angle > 90) or (angle < -90) or (self.angle != 0):
            return False

        # On motor board, 0 = full reverse, 90 is stop, 180 is full forward
        # Add 90 to produce this result given a -90 to 90 range
        data = list([speed + 90])

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x12, self.id, data)
        self.spi.writeBytes(packet)
        return True
