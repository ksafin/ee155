import spi_util as su
import packet_util as pu

class Motor:
    NEVEREST_3P9_PPR = 25.9
    NEVEREST_20_PPR = 134.4
    NEVEREST_40_PPR = 280
    NEVEREST_60_PPR = 420

    NEVEREST3P9 = 0
    NEVEREST20 = 1
    NEVEREST40 = 2
    NEVEREST60 = 3

    NO_FAULT = 0
    CURRENT_FAULT = 1
    TEMP_FAULT = 2
    DOUBLE_FAULT = 3

    def __init__(self,spi_obj,motor_id):
        self.spi = spi_obj
        self.id = motor_id
        self.cw = True
        self.PIDSet = False
        self.isPID = False
        self.ppr = 0

    # Set motor to a PWM and a given direction
    # pwm -- value from -255 to 255
    # Function ID: 0x00
    def setPWM(self, pwm):
        # Do not execute if PWM is out of range or if in PID mode
        if (pwm < -255) or (pwm > 255) or self.isPID:
            return False

        # Update internal memory of direction
        if pwm < 0:
            self.cw = False
        elif pwm > 0:
            self.cw = True

        pwm = abs(pwm)

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x00, self.id, list([pwm, int(self.cw)]))
        self.spi.writeBytes(packet)
        return True

    # Sets ppr for the motor
    # ppr -- Float value > 0
    # Function ID -- 0x1D
    def setPPR(self, ppr):
        # Do not execute if PPR is out of range
        if ppr < 0:
            return False

        # Update internally
        self.ppr = ppr

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x1D, self.id, su.FloatToBytes(ppr))
        self.spi.writeBytes(packet)
        return True

    def setPPRNeveRest(self, neverest):
        # Do not execute if invalid index is passed in
        if (neverest < 0) or (neverest > 3):
            return False

        if neverest == Motor.NEVEREST3P9:
            self.setPPR(Motor.NEVEREST_3P9_PPR)
        if neverest == Motor.NEVEREST20:
            self.setPPR(Motor.NEVEREST_20_PPR)
        if neverest == Motor.NEVEREST40:
            self.setPPR(Motor.NEVEREST_40_PPR)
        if neverest == Motor.NEVEREST60:
            self.setPPR(Motor.NEVEREST_60_PPR)

        return True

    # Sets current limit of motor, in mA
    # ilim -- Current Limit, as a value from 0 - 30,000, in mA
    # Function ID: 0x01
    def setCurrentLimit(self, ilim):
        # Do not execute if current limit is out of range
        if (ilim < 0) or (ilim > 30000):
            return False

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x01, self.id, list([su.ShortToBytes(ilim)]))
        self.spi.writeBytes(packet)
        return True


    # Read current motor speed in RPM
    # Function ID -- 0x06
    def getSpeed(self):
        # Assemble packet and write to SPI
        packet = pu.getPacket(0x06, self.id, list())
        self.spi.writeBytes(packet)

        # Read two bytes
        resp = self.spi.readBytes(2)

        # Check if the read failed, if yes, return -1
        if pu.isEmpty(resp):
            return -1

        # If read succeeded, return rpm as short
        return su.BytesToShort(resp)

    # Get motor direction as boolean
    # Returns True for cw, False for ccw
    def getDirection(self):
        return self.cw

    # Returns motor current as short, in units of mA
    # Function ID: 0x04
    def getCurrent(self):
        # Assemble packet and write to SPI
        packet = pu.getPacket(0x04, self.id, list())
        self.spi.writeBytes(packet)

        # Read two bytes
        resp = self.spi.readBytes(2)

        # Check if the read failed, if yes, return -1
        if pu.isEmpty(resp):
            return -1

        # If read succeeded, return current in mA as short
        return su.BytesToShort(resp)

    # Set PID constants, but does not enable PID
    # kp -- Proportionality constant, as a float
    # kd -- Derivative constant, as a float
    # ki -- Integral constant, as a float
    # Function ID: 0x05
    def setPID(self, kp, kd, ki):
        # Do not execute if any constant is negative
        if (kp < 0) or (kd < 0) or (ki < 0):
            return False

        self.isPID = True

        # Convert all constants to bytes
        data = list()
        data.extend(su.FloatToBytes(kp))
        data.extend(su.FloatToBytes(kd))
        data.extend(su.FloatToBytes(ki))

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x05, self.id, data)
        self.spi.writeBytes(packet)
        return True

    # PID methods for NeveRests
    def setPIDNeveRest(self, neverest):
        # Do not execute if invalid index is passed in
        if (neverest < 0) or (neverest > 3):
            return False

        if neverest == Motor.NEVEREST3P9:
            # Set PID constants
        if neverest == Motor.NEVEREST20:
            # Set PID constants
        if neverest == Motor.NEVEREST40:
            # Set PID constants
        if neverest == Motor.NEVEREST60:
            self.setPID(0.8, 0.2, 0.05)

        return True

    # Disable PID
    # Internally sets to manual Mode, alerts controller
    def disablePID(self):
        self.isPID = False

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x03, self.id, list([0]))
        self.spi.writeBytes(packet)
        return True

    # Enable PID
    # Internally sets to PID Mode, alerts controller
    # Will return false if PID constants were never set
    def enablePID(self):
        if not self.PIDSet:
            return False

        self.isPID = True

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x03, self.id, list([1]))
        self.spi.writeBytes(packet)
        return True

    # Sets speed of motor, if PID is enabled, to an RPM
    # rpm -- A short indicating speed in RPM, from -65,000 to 65,000
    # Function ID -- 0x06
    def setSpeed(self, rpm):
        # Do not execute if RPM is out of bounds or not in PID mode
        if (rpm < -65000) or (rpm > 65000) or not self.isPID:
            return False

        # Update internal memory of direction
        if rpm < 0:
            self.cw = False
        elif rpm > 0:
            self.cw = True

        # Create payload
        rpm = abs(rpm)
        data = list()
        data.extend(su.ShortToBytes(rpm))
        data.append(int(self.cw))

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x06, self.id, data)
        self.spi.writeBytes(packet)
        return True


    # Reset Encoders to 0
    # Function ID -- 0x07
    def resetEncoder(self):
        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x07, self.id, list())
        self.spi.writeBytes(packet)
        return True


    # Get Encoder Ticks since reset, as an int
    # Function ID -- 0x08
    def getEncoder(self):
        # Assemble packet and write to SPI
        packet = pu.getPacket(0x08, self.id, list())
        self.spi.writeBytes(packet)

        # Read four bytes
        resp = self.spi.readBytes(4)

        # Check if the read failed, if yes, return -1
        if pu.isEmpty(resp):
            return -1

        # If read succeeded, return rpm as int
        return su.BytesToInt(resp)


    # Rotate Motor for a fixed number of rotations at a given PWM
    # rots -- Number of rotations, as a float
    # pwm -- PWM value at which to rotate, from -255 to 255
    # Function ID -- 0x09
    def rotatePWM(self, rots, pwm):
        # Do not execute if PWM is out of range, rotations < 0, or if in PID mode
        if (pwm < -255) or (pwm > 255) or (rots < 0) or self.isPID:
            return False

        # Update internal memory of direction
         if pwm < 0:
             self.cw = False
         elif pwm > 0:
             self.cw = True

        pwm = abs(pwm)

        # Assemble payload -- pwm, then direction, then # rotations
        data = list()
        data.append(pwm)
        data.append(int(self.cw))
        data.extend(su.FloatToBytes(rots))

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x09, self.id, data)
        self.spi.writeBytes(packet)
        return True

    # Rotates a motor at a given speed for a given number of rotations
    # rots -- Number of rotations, as a float
    # rpm -- RPM value at which to rotate, from -65000 to 65000
    # Function ID: 0x0A
    def rotateSpeed(self, rots, rpm):
        # Do not execute if RPM is out of range, rotations < 0, or if not in PID mode
        if (rpm < -65000) or (rpm > 65000) or (rots < 0) or not self.isPID:
            return False

        # Update internal memory of direction
        if rpm < 0:
            self.cw = False
        elif rpm > 0:
            self.cw = True

        rpm = abs(rpm)

        # Assemble payload -- pwm, then direction, then # rotations
        data = list()
        data.extend(su.ShortToBytes(rpm))
        data.append(int(self.cw))
        data.extend(su.FloatToBytes(rots))

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x0A, self.id, data)
        self.spi.writeBytes(packet)
        return True

    # Enables Coast Mode
    # Function ID: 0x0B
    def enableCoast(self):
        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x0B, self.id, list())
        self.spi.writeBytes(packet)
        return True


    # Enables Brake Mode
    # Function ID: 0x0C
    def enableBrake(self):
        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x0C, self.id, list())
        self.spi.writeBytes(packet)
        return True


    # Sets Coast Speed
    # spd -- Coast Speed, in ms, 0-10000
    # Function ID -- 0x0D
    def setCoastSpeed(self, spd):
        # Do not execute if speed is out of range
        if (spd < 0) or (spd > 10000):
            return False

        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x0D, self.id, su.ShortToBytes(spd))
        self.spi.writeBytes(packet)
        return True

    # Stops Motor (coasting or braking)
    # Function ID -- 0x0E
    def stop(self):
        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x0E, self.id, list())
        self.spi.writeBytes(packet)
        return True

    # Release Motor
    # Function ID -- 0x0F
    def release(self):
        # Assemble packet and write to SPI, return true for success
        packet = pu.getPacket(0x0F, self.id, list())
        self.spi.writeBytes(packet)
        return True


    # Get motor fault as string
    # Compare output to internal constants to ascertain fault
    # FAULT_CURRENT, NO_FAULT, TEMP_FAULT, DOUBLE_FAULT
    # Function ID -- 0x10
    def getFault(self):
        # Assemble packet and write to SPI
        packet = pu.getPacket(0x10, self.id, list())
        self.spi.writeBytes(packet)

        # Read one byte
        resp = self.spi.readBytes(1)

        # Check if the read failed, if yes, return -1
        if pu.isEmpty(resp):
            return -1

        # If read succeeded, return first element, the fault state
        return resp[0]