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
import numpy as np
import cv2
import os
from matplotlib import pyplot as plt, cm, colors
from http.server import BaseHTTPRequestHandler, HTTPServer
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


class Car:
    # Status Variables
    mode = 0
    command = 0


class Bluetooth():
    def __init__(self):
        '''Bluetooth Communication stack'''

        self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        self.server_sock.bind(("", bluetooth.PORT_ANY))
        self.server_sock.listen(1)

        self.port = self.server_sock.getsockname()[1]

        # text mapped to commands
        self.commands = {
            "stop": 0,
            "forward": 1,
            "reverse": 2,
            "left": 3,
            "right": 4,
        }

    def connect(self):
        print("Waiting for connection on RFCOMM channel ", self.port)

        self.client_sock, self.client_info = self.server_sock.accept()
        print("Accepted connection from ", self.client_info)

    def disconnect(self):
        self.client_sock.close()
        self.server_sock.close()

    def get_data(self):
        return str(self.client_sock.recv(1024)).split("'")[1][1:-1]

    def run(self):
        Car.command = self.commands(self.get_data())

    def __del__(self):
        print("Bluetooth Disconnected.")
        self.client_sock.close()
        self.server_sock.close()


class Arduino():
    def __init__(self):
        '''Arduino Serial Communication Stack'''

        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
        self.serial.flush()

    def read(self):
        if self.serial.in_waiting > 0:
            data = self.serial.readline().decode('utf-8').rstrip().split(':')
            Car.command = data[self.mode]

    def flush(self):
        self.serial.reset_input_buffer()


class Motor():
    def __init__(self):

        # Motor control pins
        self.forward_pin = 1
        self.reverse_pin = 2
        self.left_pin = 3
        self.right_pin = 4

        # Command mapped to functions
        self.driver = {
            0: self.stop,       # Stop
            1: self.forward,    # Forward
            2: self.reverse,    # Reverse
            3: self.left,       # Left
            4: self.right,      # Right
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

    def run(self):
        self.driver[Car.command]()


class Autonomous():
    def __init__(self):

        self.detect = 0

        # Capturing video through camera
        self.camera = cv2.VideoCapture(0)

        # lane curvature mapped to commands
        self.commands = {
            "Straight": 1,
            "Left Curve": 3,
            "Right Curve": 4,
        }

        # defining variables to hold meter-to-pixel conversion
        self.ym_per_pix = 30 / 720
        self.xm_per_pix = 3.7 / 720
        # Standard lane width is 3.7 meters divided by lane width in pixels which is
        # calculated to be approximately 720 pixels not to be confused with frame height

    def detect_stop(self):

        # Convert the frame in
        # BGR(RGB color space) to
        # HSV(hue-saturation-value)
        # color space
        hsvFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
        red_lower = np.array([163, 120, 120], np.uint8)
        red_upper = np.array([189, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        kernal = np.ones((5, 5), "uint8")

        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(self.frame, self.frame, mask=red_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(
            red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.detect = 0
        Car.command = 1

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 500):
                x, y, w, h = cv2.boundingRect(contour)
                self.frame = cv2.rectangle(self.frame, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)
                print("stop")

                cv2.putText(self.frame, "Stop", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))

                self.detect = 1
                Car.command = 0

            # Program Termination
        cv2.imshow("Stop Detection in Real-TIme", self.frame)

    def detect_direction(self):

        # Apply perspective warping by calling the "perspectiveWarp()" function
        # Then assign it to the variable called (birdView)
        # Provide this function with:
        # 1- an image to apply perspective warping (frame)
        birdView, birdViewL, birdViewR, minverse = perspectiveWarp(self.frame)

        # Apply image processing by calling the "processImage()" function
        # Then assign their respective variables (img, hls, grayscale, thresh, blur, canny)
        # Provide this function with:
        # 1- an already perspective warped image to process (birdView)
        img, hls, grayscale, thresh, blur, canny = processImage(birdView)
        imgL, hlsL, grayscaleL, threshL, blurL, cannyL = processImage(
            birdViewL)
        imgR, hlsR, grayscaleR, threshR, blurR, cannyR = processImage(
            birdViewR)

        # Plot and display the histogram by calling the "get_histogram()" function
        # Provide this function with:
        # 1- an image to calculate histogram on (thresh)
        hist, leftBase, rightBase = plotHistogram(thresh)
        plt.plot(hist)

        ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(
            thresh, hist)
        plt.plot(left_fit)

        draw_info = general_search(thresh, left_fit, right_fit)

        curveRad, curveDir = measure_lane_curvature(
            ploty, left_fitx, right_fitx)

        # Filling the area of detected lanes with green
        meanPts, result = draw_lane_lines(frame, thresh, minverse, draw_info)

        deviation, directionDev = offCenter(meanPts, frame)

        # Adding text to our final image
        finalImg = addText(result, curveRad, curveDir, deviation, directionDev)

        # Displaying final image
        cv2.imshow("Final", finalImg)

    def run(self):

        # Reading the video from the
        # camera in image frames
        _, self.frame = self.camera.read()

        self.detect_stop()
        self.detect_direction()

    def readVideo(self):

        # Read input video from current working directory
        inpImage = cv2.VideoCapture(os.path.join(os.getcwd(), 'drive.mp4'))

        return inpImage

    def processImage(self, inpImage):

        # Apply HLS color filtering to filter out white lane lines
        hls = cv2.cvtColor(inpImage, cv2.COLOR_BGR2HLS)
        lower_white = np.array([0, 160, 10])
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(inpImage, lower_white, upper_white)
        hls_result = cv2.bitwise_and(inpImage, inpImage, mask=mask)

        # Convert image to grayscale, apply threshold, blur & extract edges
        gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur(thresh, (3, 3), 0)
        canny = cv2.Canny(blur, 40, 60)

        return image, hls_result, gray, thresh, blur, canny

    def perspectiveWarp(self, inpImage):

        # Get image size
        img_size = (inpImage.shape[1], inpImage.shape[0])

        # Perspective points to be warped
        src = np.float32([[590, 440],
                          [690, 440],
                          [200, 640],
                          [1000, 640]])

        # Window to be shown
        dst = np.float32([[200, 0],
                          [1200, 0],
                          [200, 710],
                          [1200, 710]])

        # Matrix to warp the image for birdseye window
        matrix = cv2.getPerspectiveTransform(src, dst)
        # Inverse matrix to unwarp the image for final window
        minv = cv2.getPerspectiveTransform(dst, src)
        birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

        # Get the birdseye window dimensions
        height, width = birdseye.shape[:2]

        # Divide the birdseye view into 2 halves to separate left & right lanes
        birdseyeLeft = birdseye[0:height, 0:width // 2]
        birdseyeRight = birdseye[0:height, width // 2:width]

        return birdseye, birdseyeLeft, birdseyeRight, minv

    def plotHistogram(self, inpImage):

        histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis=0)

        midpoint = np.int(histogram.shape[0] / 2)
        leftxBase = np.argmax(histogram[:midpoint])
        rightxBase = np.argmax(histogram[midpoint:]) + midpoint

        plt.xlabel("Image X Coordinates")
        plt.ylabel("Number of White Pixels")

        # Return histogram and x-coordinates of left & right lanes to calculate
        # lane width in pixels
        return histogram, leftxBase, rightxBase

    def slide_window_search(self, binary_warped, histogram):

        # Find the start of left and right lane lines using histogram info
        out_img = np.dstack(
            (binary_warped, binary_warped, binary_warped)) * 255
        midpoint = np.int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # A total of 9 windows will be used
        nwindows = 9
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        #### START - Loop to iterate through windows and search for lane lines #####
        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - \
                (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                          (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high),
                          (0, 255, 0), 2)
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        #### END - Loop to iterate through windows and search for lane lines #######

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Apply 2nd degree polynomial fit to fit curves
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(
            0, binary_warped.shape[0]-1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty**2 + \
            left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + \
            right_fit[1] * ploty + right_fit[2]

        ltx = np.trunc(left_fitx)
        rtx = np.trunc(right_fitx)
        plt.plot(right_fitx)

        out_img[nonzeroy[left_lane_inds],
                nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds],
                nonzerox[right_lane_inds]] = [0, 0, 255]

        # plt.imshow(out_img)
        plt.plot(left_fitx,  ploty, color='yellow')
        plt.plot(right_fitx, ploty, color='yellow')
        plt.xlim(0, 1280)
        plt.ylim(720, 0)

        return ploty, left_fit, right_fit, ltx, rtx

    def general_search(self, binary_warped, left_fit, right_fit):

        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) &
                          (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin)))

        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) &
                           (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        ploty = np.linspace(
            0, binary_warped.shape[0]-1, binary_warped.shape[0])
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        ## VISUALIZATION ###########################################################

        out_img = np.dstack(
            (binary_warped, binary_warped, binary_warped))*255
        window_img = np.zeros_like(out_img)
        out_img[nonzeroy[left_lane_inds],
                nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds],
                nonzerox[right_lane_inds]] = [0, 0, 255]

        left_line_window1 = np.array(
            [np.transpose(np.vstack([left_fitx-margin, ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                                                        ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        right_line_window1 = np.array(
            [np.transpose(np.vstack([right_fitx-margin, ploty]))])
        right_line_window2 = np.array(
            [np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
        right_line_pts = np.hstack(
            (right_line_window1, right_line_window2))

        cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

        # plt.imshow(result)
        plt.plot(left_fitx,  ploty, color='yellow')
        plt.plot(right_fitx, ploty, color='yellow')
        plt.xlim(0, 1280)
        plt.ylim(720, 0)

        ret = {}
        ret['leftx'] = leftx
        ret['rightx'] = rightx
        ret['left_fitx'] = left_fitx
        ret['right_fitx'] = right_fitx
        ret['ploty'] = ploty

        return ret

    def measure_lane_curvature(self, ploty, leftx, rightx):

        leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
        rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

        # Choose the maximum y-value, corresponding to the bottom of the image
        y_eval = np.max(ploty)

        # Fit new polynomials to x, y in world space
        left_fit_cr = np.polyfit(
            ploty*self.ym_per_pix, leftx*self.xm_per_pix, 2)
        right_fit_cr = np.polyfit(
            ploty*self.ym_per_pix, rightx*self.xm_per_pix, 2)

        # Calculate the new radii of curvature
        left_curverad = (
            (1 + (2*left_fit_cr[0]*y_eval*self.ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = (
            (1 + (2*right_fit_cr[0]*y_eval*self.ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        # Now our radius of curvature is in meters
        # print(left_curverad, 'm', right_curverad, 'm')

        # Decide if it is a left or a right curve
        if leftx[0] - leftx[-1] > 60:
            curve_direction = 'Left Curve'
        elif leftx[-1] - leftx[0] > 60:
            curve_direction = 'Right Curve'
        else:
            curve_direction = 'Straight'

        Car.command = self.commands[curve_direction]

        return (left_curverad + right_curverad) / 2.0, curve_direction

    def draw_lane_lines(self, original_image, warped_image, Minv, draw_info):

        leftx = draw_info['leftx']
        rightx = draw_info['rightx']
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']

        warp_zero = np.zeros_like(warped_image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array(
            [np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        mean_x = np.mean((left_fitx, right_fitx), axis=0)
        pts_mean = np.array(
            [np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

        newwarp = cv2.warpPerspective(
            color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

        return pts_mean, result

    def offCenter(self, meanPts, inpFrame):

        # Calculating deviation in meters
        mpts = meanPts[-1][-1][-2].astype(int)
        pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
        deviation = pixelDeviation * self.xm_per_pix
        direction = "left" if deviation < 0 else "right"

        return deviation, direction

    def addText(self, img, radius, direction, deviation, devDirection):

        # Add the radius and center position to the image
        font = cv2.FONT_HERSHEY_TRIPLEX

        if (direction != 'Straight'):
            text = 'Radius of Curvature: ' + \
                '{:04.0f}'.format(radius) + 'm'
            text1 = 'Curve Direction: ' + (direction)

        else:
            text = 'Radius of Curvature: ' + 'N/A'
            text1 = 'Curve Direction: ' + (direction)

        cv2.putText(img, text, (50, 100), font, 0.8,
                    (0, 100, 200), 2, cv2.LINE_AA)
        cv2.putText(img, text1, (50, 150), font, 0.8,
                    (0, 100, 200), 2, cv2.LINE_AA)

        # Deviation
        deviation_text = 'Off Center: ' + \
            str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
        cv2.putText(img, deviation_text, (50, 200),
                    cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 100, 200), 2, cv2.LINE_AA)

        return img

    def __del__(self):
        # self.cap.release()
        cv2.destroyAllWindows()


class WebServer(BaseHTTPRequestHandler):
    """ A special implementation of BaseHTTPRequestHander for Raspberry Pi
    """

    def __init__(self, request, client_address, server):
        super().__init__(request, client_address, server)

        # index mapped to modes
        self.available_modes = {
            0: "Manual",
            1: "IR",
            2: "Obstacle",
            3: "Voice",
            4: "GUI",
            5: "Autonomous"
        }

    def do_HEAD(self):
        """ do_HEAD() can be tested use curl command 
            'curl -I http://server-ip-address:port' 
        """
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        """ do_GET() can be tested using curl command 
            'curl http://server-ip-address:port' 
        """

        self.do_HEAD()

        if self.path == '/manual':
            Car.mode = 0
        elif self.path == '/ir':
            Car.mode = 1
        elif self.path == '/obstacle':
            Car.mode = 2
        elif self.path == '/voice':
            Car.mode = 3
        elif self.path == '/gui':
            Car.mode = 4
        elif self.path == '/autonomous':
            Car.mode = 5

        if Car.mode == 4:
            if self.path == '/stop':
                Car.command = 0
            elif self.path == '/forward':
                Car.command = 1
            elif self.path == '/reverse':
                Car.command = 2
            elif self.path == '/left':
                Car.command = 3
            elif self.path == '/right':
                Car.command = 4
        else:
            print("\n [*] ", self.available_modes[Car.mode], " Mode")

        self.send_response(200)

    def passon(self):
        pass


class MasterController:

    def __init__(self):
        '''Main framework of the car'''

        self.host_name = '192.168.0.101'
        self.host_port = 8000

        # Motor
        self.motor = Motor()

        # Arduino
        self.arduino = Arduino()

        # Bluetooth
        self.bluetooth = Bluetooth()

        # Autnomous
        self.autonomous = Autonomous()

        # Web server
        self.webserver = HTTPServer(
            (self.host_name, self.host_port), WebServer)
        print(" [*] Server started - %s:%s" % (self.host_name, self.host_port))
        self.webserver.serve_forever()

        # modes mapped to functions
        self.trigger_mode = {
            0: self.arduino.read,           # Manual
            1: self.arduino.read,           # IR
            2: self.arduino.read,           # Obstacle
            3: self.bluetooth.run,          # Voice
            4: self.webserver.passon,       # GUI
            5: self.self.autonomous.run,    # Autonomous
        }

    # Statements that need to be run continously, non-blocking
    def loop(self):
        self.trigger_mode[Car.mode]()
        self.motor.run()

    def __del__(self):
        self.webserver.server_close()
        GPIO.cleanup()


if __name__ == '__main__':
    car = MasterController()
    Car.mode = 0
    try:
        while(True):
            print(car.loop())
    except KeyboardInterrupt:
        pass

    del car
