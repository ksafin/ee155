import spi_util as su
import spidev
import time

REG_IOCONTROL = 0x0E
REG_EFR = 0x02
REG_FCR = 0x02
REG_THR = 0x00
REG_RHR = 0x00
REG_RHR = 0x00
REG_MCR = 0x04
REG_LCR = 0x03
REG_DLL = 0x00
REG_DLH = 0x01
REG_IER = 0x01
REG_LSR = 0x05
REG_IODIR = 0x0A
REG_SPR = 0x07
REG_IOSTATE = 0x0B
REG_RXLVL = 0x09
CRYSTAL_FREQ = 1843200
BAUD_RATE = 37600
RED_LED = 0x01 
GREEN_LED = 0x02
BLUE_LED = 0x04
LED_NONE = 0x00

WAIT_TICKS = 1000000000

class SpiDev:

    def __init__(self, bus, device):
        self.spi_obj = spidev.SpiDev()
        self.spi_obj.open(bus, device)
        self.spi_obj.lsbfirst = False
        self.spi_obj.max_speed_hz = 10**7
        self.debug = False
        if self.debug:
            print "Initialized SPI Bus"

    def initializeRegisters(self):
        # RESET DEVICE
        reg_val = self.readRegister(REG_IOCONTROL)
        reg_val = (reg_val | 0x08)
        self.writeRegister(REG_IOCONTROL, reg_val)
        if self.debug:
            print "Reset Device"
        
        # ENABLE FIFO
        reg_val = self.readRegister(REG_FCR)
        reg_val = (reg_val | 0x01)
        self.writeRegister(REG_FCR, reg_val)
        if self.debug:
            print "Enabled FIFO Buffer"

        # SET BAUD RATE
        reg_val = self.readRegister(REG_MCR)
        reg_val = (reg_val & 0x7F)
        self.writeRegister(REG_MCR, reg_val)
        reg_val = self.readRegister(REG_MCR)

        reg_val = self.readRegister(REG_LCR)
        reg_val = (reg_val | 0x80)
        self.writeRegister(REG_LCR, reg_val)

        # SET TO 38400 BAUD (DIVISOR = 3)
        self.writeRegister(REG_DLL, 0x03)
        self.writeRegister(REG_DLH, 0x00)

        reg_val = (reg_val & 0x7F)
        self.writeRegister(REG_LCR, reg_val)
        if self.debug:
            print "Set Baud Rate to 38400"

        # SET WORDS TO 8 BITS, NO PARITY
        reg_val = self.readRegister(REG_LCR)
        reg_val = (reg_val & 0xC0)
        reg_val = (reg_val | 0x03)
        self.writeRegister(REG_LCR, reg_val)
        if self.debug:
            print "Set Word to 8 Bit Length"

        # INITIALIZE LED PINS TO OUTPUTS
        reg_val = self.readRegister(REG_IODIR)
        reg_val = (reg_val | 0x07)
        self.writeRegister(REG_IODIR, reg_val)
        if self.debug:
            print "Initialized GPIOs"

        # ENABLE LED TO SAY "ALL GOOD"
        self.enableLED(GREEN_LED)
        if self.debug:
            print "Enabled Green LED"
    
    def writeRegister(self, addr, value):
        self.spi_obj.xfer2([addr<<3,value])
        
    def readRegister(self, addr):
        reg = 0x80 | (addr << 3)
        val = self.spi_obj.xfer2([reg, 0xFF])
        return val[1]

    def enableLED(self, led):
        self.writeRegister(REG_IOSTATE, ~led)

    def writeByte(self, byte):
        while True:
            reg_val = self.readRegister(REG_LSR)
            if not ((reg_val & 0x20) == 0):
                break
        self.writeRegister(REG_THR, byte)

    def readByte(self):
        while not self.hasData():
           pass
        if self.hasData():
            return self.readRegister(REG_RHR)
        return None

    def writeBytes(self, bytes):
        self.enableLED(RED_LED)
        for byte in bytes:
            self.writeByte(byte)
        self.enableLED(LED_NONE)
        time.sleep(0.001)

    def readBytes(self, numbytes):
        self.enableLED(BLUE_LED)
        bytes = list()
        for i in range(0,numbytes):
            bytes.append(self.readByte())
        self.enableLED(LED_NONE)
        time.sleep(0.001)
        return bytes

    def hasData(self):
        return self.readRegister(REG_RXLVL) > 0

    def enableDebug(self):
        self.debug = True

    def enableCTS(self):
        lcr = self.readRegister(REG_LCR)
        self.writeRegister(REG_LCR, 0xBF)
        reg_val = self.readRegister(REG_EFR)
        reg_val = (reg_val | 0x80)
        self.writeRegister(REG_EFR, reg_val)
        self.writeRegister(REG_LCR, lcr)

    def ping(self):
        self.writeRegister(REG_SPR, 0x55)
        if self.readRegister(REG_SPR) != 0x55:
            return False
        self.writeRegister(REG_SPR, 0xAA)
        if self.readRegister(REG_SPR) != 0xAA:
            return False
        return True
