import spidev
import math

def ReverseBits(byte):
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4)
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2)
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1)
    return byte


def BytesToHex(Bytes):
    return ''.join(["0x%02X " % x for x in Bytes]).strip()


def IntToBytes(in_int,num_bytes):
    out_bytes = []
    rest_int = in_int
    for i in range(0,num_bytes):
        p = num_bytes - 1 - i
        # print str((2**8)**p) + " current divisor \n"
        p_byte = math.floor(rest_int/((2**8)**p))
        rest_int -= p_byte * ((2**8)**p)
        # print str(p_byte) + " is the " + str(i) + ". byte \n"
        # print str(rest_int) + " remaining int \n"
        out_bytes.append(p_byte)
    return out_bytes


def BytesToInt(in_bytes):
    n = len(in_bytes)
    out_int = 0
    for i in range(0,n):
        p = n - 1 - i
        out_int += in_bytes[i]*(2**8)**p
    return out_int


def wait(in_ticks):
    ticks = 10000
    while(ticks > 0):
        ticks -= 1


def spiWrite(spi_obj,address,payload_list):
    addr_byte = (0 << 7) + (address << 3) + (0 << 1)
    # addr_byte = format((0 << 7) + (address << 3) + (0 << 1), '#010b')
    if type(payload_list) is list:
        out_list = [addr_byte] + payload_list
    else:
        out_list = [addr_byte, payload_list]
    
    # spi_obj.xfer2([addr_byte] + payload_list) #, speed_hz = 10**6, delay_us = 1, bits_per_word = 8)
    print(out_list)
    spi_obj.writebytes(out_list)


def spiRead(spi_obj,address,num_bytes):
    addr_byte = (1 << 7) + (address << 3) + (0 << 1)
    # addr_byte = format((0 << 7) + (address << 3) + (0 << 1), '#010b')
    out_val = spi_obj.xfer2([addr_byte]) # + [10**6, 0, 8])
    # out_val = spi_obj.readbytes(num_bytes)

    return out_val


def spiInit(bus,device):
    spi_obj = spidev.SpiDev()
    spi_obj.lsbfirst = False
    spi_obj.open(bus,device)

    # Set baud rate divisor
    spiWrite(spi_obj,3,(1<<7)+3)
    spiWrite(spi_obj,0,3)
    spiWrite(spi_obj,3,(0<<7)+3)

    # Enable FIFO buffering
    spiWrite(spi_obj,2,1)

    # Enable THR Interrupt for Teensy
    spiWrite(spi_obj,1,(1<<1))


    return spi_obj

def spiClose(spi_obj):
    spi_obj.close()
