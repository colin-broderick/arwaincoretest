#!/usr/bin/env python3

import time
import board
import adafruit_rfm9x
import zmq
import sys
import busio
import numpy as np
from tornado.ioloop import PeriodicCallback
from digitalio import DigitalInOut
import time


class Tx:
    def __init__(self, packet_size=6):
        self.packet_size = packet_size
        self.alert_flag = 0
        self.rfm = self.radio()
        self.x = 0
        self.y = 0
        self.z = 0
        self.falling = 0
        self.entangled = 0
        self.stance = "inactive"

    def store_alert(self, x, y, z, falling, entangled, stance):
        self.x = x
        self.y = y
        self.z = z
        self.falling = falling
        self.entangled = entangled
        self.stance = stance

        alert_flag = 0
        if self.falling:
            alert_flag = alert_flag | 1
        elif self.entangled:
            alert_flag = alert_flag | 2

        if self.stance == "inactive":
            alert_flag = alert_flag | (0<<2)
        elif self.stance == "walking":
            alert_flag = alert_flag | (1<<2)
        elif self.stance == "searching":
            alert_flag = alert_flag | (2<<2)
        elif self.stance == "crawling":
            alert_flag = alert_flag | (3<<2)
        elif self.stance == "running":
            alert_flag = alert_flag | (4<<2)
        elif self.stance == "unknown":
            alert_flag = alert_flag | (5<<2)

        self.alert_flag = alert_flag

        return True

    def encode_message(self):
        x1 = np.float16(self.x).tobytes()
        y1 = np.float16(self.y).tobytes()
        z1 = np.float16(self.z).tobytes()
        flag = np.float16(self.alert_flag).tobytes()
        return x1 + y1 + z1 + flag

    def tx(self):
        """
        Enable transmitter mode.
        """
        #cb = PeriodicCallback(self.send, 1000)
        #cb.start()
        #print(cb.is_running())
        pass

    def send(self):
        print("hello")
        """
        Transmit the most recently-receceived position update.
        """
        if True:
            print("sending")
            message = self.encode_message()
            self.rfm.send(message)
            print("message tx", message, len(message), "bytes")

    def radio(self):
        """
        Create adafruit LoRa object.
        """
        ## These are the defaults from the board demo but found no reason to change them.
        cs = DigitalInOut(board.CE1)
        reset = DigitalInOut(board.D25)
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        frequency = 868.0
        rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, frequency)
        rfm9x.tx_power = 23
        #rfm9x.spreading_factor = 12
        #rfm9x.signal_bandwidth = 125000
        #rfm9x.coding_rate = 8
        rfm9x.enable_crc = True
        return rfm9x


def main():
    ## Create radio
    lora = Tx()
    lora.tx()

    ## Create socket
    context = zmq.Context()
    print("Waiting for data connection")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5556")
    socket.send("accept".encode("ascii"))
    
    response = "messagesent".encode("ascii")

    while True:
        ## Wait for incoming message
        message = socket.recv()
        message = message.decode("ascii")
        if message == "stop":
            break
        
        ## Process message
        message = message.split(",")
        x = float(message[0])
        y = float(message[1])
        z = float(message[2])
        falling = 1 if message[3] == "f" else 0
        entangled = 1 if message[4] == "e" else 0
        stance = message[5]
        lora.store_alert(x, y, z, falling, entangled, stance)
        lora.send()
        
        ## Send acknowledgement
        socket.send(response)


if __name__ == "__main__":
    main()
