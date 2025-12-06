import time

import serial
from time import sleep


class Communicator:
    """
    The Communicator class handles a serial connection from the Pi to the ESP32.
    To set this up, the two devices need to be connected together like so:
       Raspberry Pi          ESP32 Dev Board
    +--------------+        +---------------+
    |    Pin 6 (GND)  ------  GND           |
    |    Pin 8 (TX)   ------  RX2 (GPIO 16) |
    |    Pin 10 (RX)  ------  TX2 (GPIO 17) |
    +--------------+        +---------------+
    Additionally, the Pi's serial monitor must be disabled:
    sudo raspi-config -> Interface Options -> I6 Serial Port. Disable login shell but enable serial hardware
    """
    def __init__(self, port="/dev/serial0", baud=115200, debug=False):
        self.port = port
        self.baudrate = baud
        self.serial = None
        self.debug = debug
        self.connect()

    def connect(self):
        while self.serial is None:
            try:
                self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
                if self.debug: print("Opened Serial Port!")
                sleep(2)
            except serial.SerialException as e:
                self.serial = None
                time.sleep(5)
                print("Could not open serial port %s" % e)

    def send(self, text: str):
        if self.serial is not None:
            data_string = text + "\n"
            try:
                self.serial.write(data_string.encode('utf-8'))
            except Exception as e:
                print(e)
                self.serial = None
                self.connect()
            if self.debug: print(data_string.strip())

    def finish(self):
        if self.serial is not None:
            self.serial.close()
