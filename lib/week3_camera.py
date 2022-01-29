import cv2 
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from picarx import *
import time
import math 
import logging

class Navigate: 
    def __init__(self) -> None:
        pass
    def detect_edges(frame):

        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        # frame = cv2.GaussianBlur(frame, (3,3), 0)
        cv2.imwrite("1.jpg", frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imwrite("/home/pi/picar-x/hsv.jpg", hsv)

        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        cv2.imwrite("/home/pi/picar-x/blue_mask.jpg", mask)
        # detect edges
        edges = cv2.Canny(mask, 200, 400)
        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)
        # only focus bottom half of the screen
        polygon = np.array([[
            (100, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (100, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges
    
    def detect_line_segments(self, cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4)
        return line_segments
    
    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=2):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image
        
    def steering_angle(self, edges, lane_lines):
        # height, width = edges.shape
        # _, _, left_x2, _ = lane_lines[0][0]
        # _, _, right_x2, _ = lane_lines[1][0]
        # camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        # mid = int(width / 2 * (1 + camera_mid_offset_percent))
        # x_offset = (left_x2 + right_x2) / 2 - mid

        # # find the steering angle, which is angle between navigation direction to end of center line
        # y_offset = int(height / 2)

        # angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        # angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        # steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
        ll = np.array(lane_lines)
        # print(ll[:,:,0])
        coefs = np.polyfit(ll[:,:,0].ravel(), ll[:,:,1].ravel(), 1)
        ang=math.atan(-coefs[1]/coefs[0])
        ang=(ang* 180.0)/math.pi -90
        print(ang)
        return ang

if __name__=="__main__":
    px = Picarx()
    px.set_camera_servo2_angle(-45)
    nv = Navigate()
    for i in range(1000):
        edges = nv.detect_edges()
        cropped_edges = nv.region_of_interest(edges)
        lane_lines = nv.detect_line_segments(cropped_edges)
        print(lane_lines)
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()    
        lane_lines_image = nv.display_lines(frame, lane_lines)
        cv2.imwrite("lane_lines.jpg", lane_lines_image)
        steering_angle = nv.steering_angle(edges, lane_lines)
        if abs(steering_angle)>30:
            steering_angle=30
        px.set_dir_servo_angle(steering_angle)
        px.backward(0)
        
    