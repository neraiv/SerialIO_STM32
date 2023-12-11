import SerialIO
import time

SerialIO.list_ports()

sio = SerialIO(baudrate = 115200, portName = "COM5")

while(1):
    sio.setPin(0x00AA,0)
    time.sleep(1)
    sio.setPin(0x00AA,1)
    time.sleep(1)