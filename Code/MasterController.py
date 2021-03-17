#!/usr/bin/env python3
# ================================================================================
'''
    File name: MasterController.py
    Author: Jazib Dawre <jazib980@gmail.com>
    Version: 1.0.0
    Date created: 17/03/2021
    Description: Master control code for an Autonomous Car
    Python Version: 3+ [32-bit] (Raspberry Pi only)
    Optional Repositories: None
    License: GPL-3.0 License

    Copyright (C) 2020 Jazib Dawre

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
# ================================================================================
import RPi.GPIO as GPIO
import serial
import bluetooth
# ================================================================================
GPIO.setmode(GPIO.BOARD)
# ================================================================================
'''
    Serial Data format:
        MANUAL : INFRARED : OBSTACLE

    Command Index:
        0   -   Stop
        1   -   Forward
        2   -   Reverse
        3   -   Left
        4   -   Right

    Mode Index:
        0   -   MANUAL
        1   -   INFRARED
        2   -   OBSTACLE
        3   -   VOICE
        4   -   GUI
        5   -   AUTONOMOUS
'''
# ================================================================================


class Bluetooth:
    def __init__(self):
        '''Bluetooth Communication stack'''
        self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        self.server_sock.bind(("", bluetooth.PORT_ANY))
        self.server_sock.listen(1)

        self.port = self.server_sock.getsockname()[1]

        self.uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

    def connect(self):
        bluetooth.advertise_service(self.server_sock, "Autonomous Car", service_id=self.uuid,
                                    service_classes=[
                                        self.uuid,
                                        bluetooth.SERIAL_PORT_CLASS
                                    ],
                                    profiles=[bluetooth.SERIAL_PORT_PROFILE])

        print("Waiting for connection on RFCOMM channel ", self.port)

        self.client_sock, selfclient_info = self.server_sock.accept()
        print("Accepted connection from ", self.client_info)

    def disconnect(self):
        self.client_sock.close()
        self.server_sock.close()

    def getData(self):
        return self.client_sock.recv(1024)

    def __del__(self):
        print("Bluetooth Disconnected.")
        self.client_sock.close()
        self.server_sock.close()


class Arduino:
    def __init__(self):
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
        self.serial.flush()

    def read(self):
        if self.serial.in_waiting > 0:
            data = self.serial.readline().decode('utf-8').rstrip().split(':')
            self.command = data[self.mode]

    def flush(self):
        self.serial.reset_input_buffer()


class Motor:
    def __init__(self):

        # Motor control pins
        self.forward_pin = 1
        self.reverse_pin = 2
        self.left_pin = 3
        self.right_pin = 4

        # Command mapped to functions
        self.driver = {
            0: self.stop,              # Stop
            1: self.forward,           # Forward
            2: self.reverseforward,    # Reverse
            3: self.leftforward,       # Left
            4: self.rightforward,      # Right
        }

        # Setup
        GPIO.output(self.forward_pin, GPIO.OUT)
        GPIO.output(self.reverse_pin, GPIO.OUT)
        GPIO.output(self.left_pin, GPIO.OUT)
        GPIO.output(self.right_pin, GPIO.OUT)

    def forward(self):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.reverse_pin, GPIO.LOW)
        GPIO.output(self.left_pin, GPIO.LOW)
        GPIO.output(self.right_pin, GPIO.LOW)

    def reverse(self):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.reverse_pin, GPIO.HIGH)
        GPIO.output(self.left_pin, GPIO.LOW)
        GPIO.output(self.right_pin, GPIO.LOW)

    def left(self):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.reverse_pin, GPIO.LOW)
        GPIO.output(self.left_pin, GPIO.HIGH)
        GPIO.output(self.right_pin, GPIO.LOW)

    def right(self):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.reverse_pin, GPIO.LOW)
        GPIO.output(self.left_pin, GPIO.LOW)
        GPIO.output(self.right_pin, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.reverse_pin, GPIO.LOW)
        GPIO.output(self.left_pin, GPIO.LOW)
        GPIO.output(self.right_pin, GPIO.LOW)

    def run(self, command):
        self.driver[command]()


class MasterController:

    def __init__(self):
        '''Main framework of the car'''

        # Status Variables
        self.mode = 0
        self.command = 0
        self.read = 0

        # modes mapped to functions
        self.select_modes = {
            0: self.manual_mode,                    # Manual
            1: self.ir_mode,                        # IR
            2: self.obstacle_mode,                  # Obstacle
            3: self.voice_controlled_mode,          # Voice
            4: self.gui_controlled_mode,            # GUI
            5: self.autonomous_controlled_mode,     # Autonomous
        }

        # Motor
        self.motor = Motor()

        # Arduino
        self.arduino = Arduino()

        # Bluetooth
        self.bluetooth = Bluetooth()

    # Statements that need to be run continously, non-blocking
    def loop(self):
        self.motor.run(self.command)

    def set_mode(self):
        self.mode = 0   # from webserver
        self.select_modes[self.mode]()

    # Mode definitions

    def manual_mode(self):  # Left to implement
        print("\n [*] Manual Mode")  # transfer these to webserver
        self.arduino.read()
        self.command = self.arduino.command

    def ir_mode(self):  # Left to implement
        print("\n [*] IR Mode")
        self.arduino.read()
        self.command = self.arduino.command

    def obstacle_mode(self):  # Left to implement
        print("\n [*] Obstacle Mode")
        self.arduino.read()
        self.command = self.arduino.command

    def voice_controlled_mode(self):  # Left to implement
        print("\n [*] Voice Controlled Mode")
        self.bluetooth.connect()

    def gui_mode(self):  # Left to implement
        print("\n [*] GUI Mode")

    def autonomous_controlled_mode(self):  # Left to implement
        print("\n [*] Autnomous Mode")

    def __del__(self):
        GPIO.cleanup()


if __name__ == '__main__':
    bl = Bluetooth()
    bl.connect()

    input()

    bl.disconnect()
    del bl
