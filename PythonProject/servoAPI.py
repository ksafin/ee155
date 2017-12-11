
import spi_util as su

class Servo:
    def __init__(self,spi_obj,servo_id):
        self.spi = spi_obj
        self.id = servo_id


    # Set Servo Angle
    #define FID_SETANG    0x11  // Set Servo Angle
    def SetAngle(self, angle_val):
        fun_id = 17
        val_bytes = su.IntToBytes(pwm_val,2)
        byte_list = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, byte_list)


    # Set Continuous Servo Speed
    #define FID_SETSPD    0x12  // Set Continuous Servo Speed
    def SetAngle(self, rpm_int):
        fun_id = 18
        val_bytes = su.IntToBytes(rpm_int,2)
        byte_list = [self.id, (fun_id << 2) + 2] + val_bytes

        self.spi.spiWrite(0, byte_list)
