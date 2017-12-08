# import spidev
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
    for i in range(0,in_int):
        p = in_int - 1 - i
        p_byte = math.floor(rest_int/(8**p))
        rest_int -= p_byte * (8**p)
        out_bytes.append(p_byte)
    return out_bytes


def spiInit(bus,device):
    spi_obj = spidev.SpiDev()
    spi_obj.open(bus,device)
    return spi_obj

def spiClose(spi_obj):
    spi_obj.close()

def spiWrite(spi_obj,address,payload_list):
    addr_byte = (0 << 7) + (address << 3) + (0 << 1)
    # addr_byte = format((0 << 7) + (address << 3) + (0 << 1), '#010b')
    spi_obj.xfer2([addr_byte] + payload_list + [10**6, 0, 8])

def spiRead(spi_obj,address,num_bytes):
    addr_byte = (1 << 7) + (address << 3) + (0 << 1)
    # addr_byte = format((0 << 7) + (address << 3) + (0 << 1), '#010b')
    spi_obj.xfer2([addr_byte] + [10**6, 0, 8])
    spi_obj.readbytes(num_bytes)
