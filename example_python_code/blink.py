
import SerialIO
import time

SerialIO.list_ports() # Lists ports avaliable using pyserial

sio = SerialIO.SerialIO(baudrate = 115200, portName = "COM5") # Connects the port selected

## Example blink application.
while(1):
    sio.setDigital(0x00AA, SerialIO.PIN_State.HIGH)
    time.sleep(1)
    sio.setDigital(0x00AA, SerialIO.PIN_State.LOW)
    time.sleep(1)
