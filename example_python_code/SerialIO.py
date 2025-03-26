import serial
import serial.tools.list_ports
import time
import struct
import threading
import queue

def list_ports():
    print("Listing ports...")
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(f"{port}: {desc} [{hwid}]")

def print_bytes(msg):
    print(f"Binary: {bin(int.from_bytes(msg, 'big'))}")

def complete_to_6_bytes(msg):
    return msg.ljust(6, b'\x00') if len(msg) < 6 else msg[:6]

class PIN_State:
    LOW = 0
    HIGH = 1

class PinDigital:
    def __init__(self, id: int):
        self.id = id
        self._state = PIN_State.LOW
        self._lock = threading.Lock()

    @property
    def state(self):
        with self._lock:
            return self._state

    @state.setter
    def state(self, value):
        with self._lock:
            self._state = value

class PinAnalog:
    def __init__(self, id: int, min: int = 0, max: int = 255, 
                 volt_high: float = 5.0, volt_low: float = 0.0):
        self.id = id
        self.min = min
        self.max = max
        self._value = 0
        self.volt_high = volt_high
        self.volt_low = volt_low
        self._lock = threading.Lock()

    @property
    def value(self):
        with self._lock:
            return self._value

    @value.setter
    def value(self, val):
        with self._lock:
            self._value = max(min(val, self.max), self.min)

    def convert_voltage(self, voltage):
        return int((voltage - self.volt_low) / (self.volt_high - self.volt_low) * (self.max - self.min))

class SioValue:
    def __init__(self, id, initial_value, value_type: type):
        self.id = id
        self._value = initial_value
        self.type = value_type
        self._lock = threading.Lock()

    @property
    def value(self):
        with self._lock:
            return self._value

    @value.setter
    def value(self, val):
        with self._lock:
            if isinstance(val, self.type):
                self._value = val
            else:
                raise TypeError(f"Expected {self.type}, got {type(val)}")

class SerialIO:
    def __init__(self, baudrate: int, port_name: str):
        self.port = serial.Serial(port=port_name, baudrate=baudrate, timeout=1)
        self._command_queue = queue.Queue()
        self._running = False

        self.pins_digital = []
        self.pins_analog = []
        self.sio_values = []

        self._rx_buffer = bytearray()
        self._start_threads()

    def _start_threads(self):
        self._running = True
        self._tx_thread = threading.Thread(target=self._process_commands)
        self._rx_thread = threading.Thread(target=self._receive_loop)
        self._tx_thread.start()
        self._rx_thread.start()

    def add_pin(self, pin):
        if isinstance(pin, (PinDigital, PinAnalog)):
            getattr(self, f'pins_{type(pin).__name__[3:].lower()}').append(pin)

    def add_sio_value(self, sio_value: SioValue):
        self.sio_values.append(sio_value)

    def set_digital(self, pin: PinDigital, state: int):
        msg = pin.id.to_bytes(2, 'big') + state.to_bytes(1, 'big')
        self._command_queue.put(complete_to_6_bytes(msg))
        pin.state = state

    def set_analog(self, pin: PinAnalog, value: int):
        pin.value = value
        msg = pin.id.to_bytes(2, 'big') + pin.value.to_bytes(2, 'big')
        self._command_queue.put(msg)

    def set_value(self, sio_value: SioValue, value):
        sio_value.value = value
        msg = sio_value.id.to_bytes(2, 'big') + struct.pack('f', sio_value.value)
        self._command_queue.put(msg)

    def _process_commands(self):
        while self._running:
            try:
                msg = self._command_queue.get(timeout=0.1)
                self.port.write(msg)
            except queue.Empty:
                continue

    def _receive_loop(self):
        while self._running:
            if self.port.in_waiting:
                self._rx_buffer += self.port.read_all()
                self._parse_messages()
            time.sleep(0.01)

    def _parse_messages(self):
        while len(self._rx_buffer) >= 6:
            chunk, self._rx_buffer = self._rx_buffer[:6], self._rx_buffer[6:]
            msg_id = int.from_bytes(chunk[:2], 'big')
            payload = chunk[2:]

            # Update digital pins
            for pin in self.pins_digital:
                if pin.id == msg_id:
                    pin.state = payload[0]
                    break
            else:
                # Update analog pins
                for pin in self.pins_analog:
                    if pin.id == msg_id:
                        pin.value = int.from_bytes(payload[:2], 'big')
                        break
                else:
                    # Update SioValues
                    for val in self.sio_values:
                        if val.id == msg_id:
                            val.value = struct.unpack('f', payload[:4])[0]
                            break

    def close(self):
        self._running = False
        self._tx_thread.join()
        self._rx_thread.join()
        self.port.close()

# Example usage
if __name__ == "__main__":
    sio = SerialIO(9600, "COM3")
    
    led = PinDigital(1)
    sensor = PinAnalog(2)
    temp_value = SioValue(3, 0.0, float)
    
    sio.add_pin(led)
    sio.add_pin(sensor)
    sio.add_sio_value(temp_value)
    
    # Non-blocking operations
    sio.set_digital(led, PIN_State.HIGH)
    sio.set_analog(sensor, 128)
    sio.set_value(temp_value, 25.5)
    
    time.sleep(2)
    print(f"LED State: {led.state}")
    print(f"Sensor Value: {sensor.value}")
    print(f"Temperature: {temp_value.value}")
    
    sio.close()