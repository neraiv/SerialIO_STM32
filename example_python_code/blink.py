import SerialIO
import time

SerialIO.list_ports() # Lists ports avaliable using pyserial

sio = SerialIO.SerialIO(baudrate = 115200, portName = "COM5") # Connects the port selected

## Example blink application.
while(1):
    sio.setPin(0x00AA,0)
    time.sleep(1)
    sio.setPin(0x00AA,1)
    time.sleep(1)
