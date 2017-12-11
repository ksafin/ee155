import time
import spidev

spi = spidev.SpiDev()
spi.open(0,1)

spi.max_speed_hz = 1000

try:
    while True:
        resp = spi.xfer2([0xAA])
        print resp[0]
        time.sleep(1)
    #end while
except KeyboardInterrupt:
    spi.close()
#end try