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

        bluetooth.advertise_service(self.server_sock, "Autonomous Car", service_id=self.uuid,
                                    service_classes=[
                                        self.uuid,
                                        bluetooth.SERIAL_PORT_CLASS
                                    ],
                                    profiles=[bluetooth.SERIAL_PORT_PROFILE])

        print("Waiting for connection on RFCOMM channel ", self.port)

        self.client_sock, selfclient_info = self.server_sock.accept()
        print("Accepted connection from ", self.client_info)

    def getData(self):
        return client_sock.recv(1024)

    def __del__(self):
        print("Bluetooth Disconnected.")
        self.client_sock.close()
        self.server_sock.close()


class MasterController:

    def __init__(self):
        '''Main framework of the rover'''

        # Current Mode
        self.mode = 0
        self.command = 0

        # Command mapped to Motor Driver pins
        self.driver = {
            0: 1    # Stop
            1: 2    # Forward
            2: 3    # Reverse
            3: 4    # Left
            4: 5    # Right
        }

        # Motor control pins
        for command, pin in enumerate(self.driver):
            GPIO.setup(self.driver[pin], GPIO.OUT)

        # Serial to arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.flush()

   def loop(self):

        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').rstrip().split(':')
            if self.mode < 3:
                self.command = data[self.mode]

        if(self.command):
            GPIO.output(self.driver[self.command], GPIO.HIGH)
        else:
            for command, pin in enumerate(self.driver):
                GPIO.output(self.driver[pin], GPIO.LOW)

    def webserver(self):
        if(self.mode == 0):
            print("\nManual Mode")
        elif(self.mode == 1):
            print("\nIR Mode")
        elif(self.mode == 2):
            print("\nObstacle Mode")
        if(self.mode == 3):
            print("\nVoice Controlled Mode")
            voice_controlled_mode()
        if(self.mode == 4):
            print("\nGUI Mode")
            gui_controlled_mode()
        if(self.mode == 5):
            print("\nAutnomous Mode")
            autonomous_controlled_mode()

    # Mode definitions

    def voice_controlled_mode(self):  # Left to implement
        pass

    def autonomous_controlled_mode(self):  # Left to implement
        pass

    def __del__(self):
        GPIO.cleanup()
