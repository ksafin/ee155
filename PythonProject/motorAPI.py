
import spi_util as su

class Motor:
    def __init__(self,spi_obj,motor_id):
        self.spi = spi_obj
        self.id = motor_id

        self.PID = False

    # Set pwm of motor motor_id on board with spi_obj to 12 Bit int pwm_val
    #define FID_SETPWM   0x00  // Set Motor PWM
    def setPWM(self, pwm_val):
        fun_id = 0
        val_bytes = su.IntToBytes(pwm_val,2)
        byte_list = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, byte_list)


    # Set current limit of motor motor_id on board with spi_obj to cur_val in units of 1 mA
    #define FID_SETILIM   0x01  // Set Motor Current Limit
    def setCurrentLimit(self, cur_val):
        fun_id = 1
        val_bytes = su.IntToBytes(cur_val,2)
        in_bytes = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, in_bytes)


    # Read current motor speed in RPM
    #define FID_GETSPD    0x02  // Get Motor Speed (RPM)
    def getSpeed(self):
        fun_id = 2
        in_bytes = [self.id,(fun_id << 2) + 0]
        self.spi.spiWrite(0,in_bytes)
        out_bytes = self.spi.spiRead(0, 2)
        out_speed = su.BytesToInt(out_bytes)
        return out_speed


    # Get motor direction as boolean
    #define FID_GETDIR    0x03  // Get Motor Direction
    def getDirection(self):
        fun_id = 3
        in_bytes = [self.id,(fun_id << 2) + 0]
        self.spi.spiWrite(0,in_bytes)
        out_bytes = self.spi.spiRead(0, 1)
        return (out_bytes[0] > 0)


    # Gwt motor current as int in units of 1 mA
    #define FID_GETCUR    0x04  // Get Motor Current
    def getCurrent(self):
        fun_id = 4
        in_bytes = [self.id,(fun_id << 2) + 0]
        self.spi.spiWrite(0,in_bytes)
        out_bytes = self.spi.spiRead(0, 2)
        out_current = su.BytesToInt(out_bytes)
        return out_current


    # Set PID constants
    #define FID_SETPID    0x05  // Set PID Constants
    def enablePID(self, kp_val, ki_val, kd_val):
        fun_id = 5
        val_bytes = [kp_val,ki_val,kd_val]
        in_bytes = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, in_bytes)


    # Set Motor speed as an RPM int value
    #define FID_SETSPD    0x06  // Set Motor Speed (RPM)
    def setSpeed(self, rpm_val):
        fun_id = 6
        val_bytes = su.IntToBytes(rpm_val,2)
        byte_list = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, byte_list)


    # Reset Encoders
    #define FID_RESENC    0x07  // Reset Motor Encoder
    def ResetEncoder(self):
        fun_id = 7
        byte_list = [self.id, (fun_id << 2) + 2]

        self.spi.spiWrite(0, byte_list)


    # Get encoder ticks as int
    #define FID_GETENC    0x08  // Get Motor Encoder Ticks
    def getEncoder(self):
        fun_id = 8
        in_bytes = [self.id,(fun_id << 2) + 0]

        out_bytes = self.spi.spiRead(0, 2)
        out_ticks = su.BytesToInt(out_bytes)
        return out_ticks


    # Rotate motor for a fixed number of rotations at given PWM
    #define FID_ROTPWM    0x09  // Rotate for some Revs at given PWM
    def rotatePWM(self, rotation_num, pwm_val):
        fun_id = 9
        val_bytes = su.IntToBytes(pwm_val,2)
        byte_list = [self.id, (fun_id << 2) + 2] + [rotation_num] + val_bytes

        self.spi.spiWrite(0, byte_list)



    #define FID_ROTSPD    0x0A  // Rotate for some revs at given RPM
    def rotateSpeed(self, rotation_num, rpm_val):
        fun_id = 10
        val_bytes = su.IntToBytes(rpm_val,2)
        byte_list = [self.id, (fun_id << 2) + 3] + [rotation_num] + val_bytes

        self.spi.spiWrite(0, byte_list)


    # Enable coast mode
    #define FID_COAST     0x0B  // Enable Coast
    def enableCoast(self, rotation_num, rpm_val):
        fun_id = 11
        val_bytes = su.IntToBytes(rpm_val,2)
        byte_list = [self.id, (fun_id << 2) + 0]

        self.spi.spiWrite(0, byte_list)


    # Enable brake mode
    #define FID_BRAKE     0x0C  // Enable Brake
    def enableCoast(self):
        fun_id = 12
        byte_list = [self.id, (fun_id << 2) + 0]

        self.spi.spiWrite(0, byte_list)


    # Set coast speed
    #define FID_CSPD      0x0D  // Set Coast Speed
    def setCoastSpeed(self, rpm_val):
        fun_id = 13
        val_bytes = su.IntToBytes(rpm_val,2)
        byte_list = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, byte_list)


    # Stop Motor
    #define FID_STOP      0x0E  // Stop Motor
    def stop(self,brake):
        fun_id = 14

        if(brake):
            val_bytes = [1]
        else:
            val_bytes = [0]

        byte_list = [self.id, (fun_id << 2) + 0] + val_bytes

        self.spi.spiWrite(0, byte_list)


    # Release Motor
    #define FID_REL       0x0F  // Release Motor
    def release(self):
        fun_id = 15
        byte_list = [self.id, (fun_id << 2) + 0]

        self.spi.spiWrite(0, byte_list)


    # Get motor fault as string
    #define FID_FAULT     0x10  // Get Motor Fault
    def getFault(self):
        fun_id = 16
        in_bytes = [self.id,(fun_id << 2) + 0]
        self.spi.spiWrite(0, in_bytes)
        out_byte = self.spi.spiRead(0, 1)
        if (out_byte == 0):
            out_fault = 'NOFAULT'
        elif (out_byte == 1):
            out_fault = 'THERMALFAULT'
        elif (out_byte == 2):
            out_fault = 'CURRENTFAULT'
        elif (out_byte == 3):
            out_fault = 'VOLTAGEFAULT'
        return out_fault
