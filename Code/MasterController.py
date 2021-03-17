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
# ================================================================================
GPIO.setmode(GPIO.BOARD)
# ================================================================================
'''
    Serial Data format:
        MANUAL : INFRARED : OBSTACLE
    
    Index:
        MANUAL : INFRARED : OBSTACLE
          0         0          0         -   Forward
          1         1          1         -   Stop
          2         2                    -   Reverse
          3         3                    -   Right
          4         4                    -   Left
'''
# ================================================================================


class MasterController:

    def __init__(self):
        '''Main framework of the rover'''

        # Current Mode
        self.mode = 0

        # Motor Driver pins
        self.forward = 24
        self.backward = 28
        self.left = 14
        self.right = 14

        # Motor control pins
        GPIO.setup(forward, GPIO.OUT)
        GPIO.setup(backward, GPIO.OUT)
        GPIO.setup(left, GPIO.OUT)
        GPIO.setup(right, GPIO.OUT)

        # Serial to arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.flush()

    def loop():

        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').rstrip().split(':')

        if(GPIO.input(2) == GPIO.HIGH):
            print("Manual Mode")
            manual_mode()
        elif(GPIO.input(3) == GPIO.HIGH):
            # print("IR Mode")
            line_follower_mode()
        elif(GPIO.input(4) == GPIO.HIGH):
            print("Voice Controlled Mode")
            voice_controlled_mode()
        if(GPIO.input(5) == GPIO.HIGH):
            print("Obstacle Mode")
            obstacle_detection_mode()

    def obstacle_detection_mode():

        # Only for testing
        print("Obstacle IR: ")
        print(obstacle_sensor_state)

        if (obstacle_sensor_state > 500):
            print("OBSTACLE!\nStop")
            GPIO.output(forward, GPIO.LOW)
            GPIO.output(backward, GPIO.LOW)
            GPIO.output(left, GPIO.LOW)
            GPIO.output(right, GPIO.LOW)

    def manual_mode():

        left_sensor_state = GPIO.input(38)
        right_sensor_state = GPIO.input(34)
        forward_sensor_state = GPIO.input(44)

        # Only for testing
        print("Left IR: ")
        print(left_sensor_state)
        print(" | Right IR: ")
        print(right_sensor_state)
        print(" | Forward IR: ")
        print(forward_sensor_state)

        if (forward_sensor_state == GPIO.HIGH):
            print("Forward")
            GPIO.output(right, GPIO.HIGH)
            GPIO.output(left, GPIO.HIGH)
            if (left_sensor_state == GPIO.HIGH):
                print("Turning Left")
                GPIO.output(forward, GPIO.LOW)
                GPIO.output(backward, GPIO.HIGH)

            if (right_sensor_state == GPIO.HIGH):
                print("Turning Right")
                GPIO.output(forward, GPIO.HIGH)
                GPIO.output(backward, GPIO.LOW)

        else:
            print("Stop")
            GPIO.output(forward, GPIO.LOW)
            GPIO.output(backward, GPIO.LOW)
            GPIO.output(left, GPIO.LOW)
            GPIO.output(right, GPIO.LOW)

    def voice_controlled_mode():  # Left to implement
        pass

    def line_follower_mode():

        left_sensor_state = analogRead(left_sensor_pin)
        right_sensor_state = analogRead(right_sensor_pin)

        # Only for testing
        print("Left IR: ")
        print(left_sensor_state)
        print(" | Right IR: ")
        print(right_sensor_state)

        if (right_sensor_state < 500 and left_sensor_state > 500):
            # print("Turning Left")
            GPIO.output(forward, GPIO.LOW)
            GPIO.output(backward, GPIO.HIGH)
            # Change to GPIO.LOW for friction turning
            GPIO.output(left, GPIO.HIGH)
            GPIO.output(right, GPIO.HIGH)

        if (right_sensor_state > 500 and left_sensor_state < 500):
            # print("Turning Right")
            GPIO.output(forward, GPIO.HIGH)
            GPIO.output(backward, GPIO.LOW)
            GPIO.output(left, GPIO.HIGH)
            # Change to GPIO.LOW for friction turning
            GPIO.output(right, GPIO.HIGH)

        if (right_sensor_state < 500 and left_sensor_state < 500):
            # print("Forward")
            GPIO.output(backward, GPIO.LOW)  # Change to GPIO.HIGH for 4WD
            GPIO.output(forward, GPIO.LOW)  # Change to GPIO.HIGH for 4WD
            GPIO.output(right, GPIO.HIGH)
            GPIO.output(left, GPIO.HIGH)

        if (right_sensor_state > 500 and left_sensor_state > 500):
            # print("Stop")
            GPIO.output(forward, GPIO.LOW)
            GPIO.output(backward, GPIO.LOW)
            GPIO.output(left, GPIO.LOW)
            GPIO.output(right, GPIO.LOW)

    GPIO.cleanup()
