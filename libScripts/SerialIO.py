import serial 
import serial.tools.list_ports 

import time

import struct

def list_ports():
        ports = serial.tools.list_ports.comports()

        increment = 0
        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))
            increment += 1

def print_bytes(msg):
    # Convert the byte to its integer representation
    byte_as_int = int.from_bytes(msg, byteorder='big')
    binary_representation = "{0:b}".format(byte_as_int)
    print(binary_representation) 
     
def complete_to_6_bytes(msg):
    diff = 6 - len(msg)
    
    if diff > 0:
        for i in range(diff):
            msg += (0).to_bytes(1,"big")

    elif diff < 0:
        print("ERROR")
        
    return msg

class SerialIO():
    def __init__(self, baudrate: int, portName: str) -> None:
        self.port = serial.Serial(port="COM5",baudrate=baudrate,timeout=1)
        self.port.close()
        self.port.open()

    def setPin(self, id: int, state: int):
        msg = id.to_bytes(2,byteorder="big") + state.to_bytes(1,byteorder="big")
        msg = complete_to_6_bytes(msg)
        self.port.write(msg)
    
    def setPort(self, id: int, value: float):
        msg = id.to_bytes(2,byteorder="big") + struct.pack('f',value) 
        self.port.write(msg)
