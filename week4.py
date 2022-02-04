import concurrent.futures
import requests
import threading
import time
import cv2
import numpy as np
from picarx import Picarx

thread_local = threading.local()

class Bus:
    def __init__(self, message):
        self.message = message

    def reader(self):
        return self.message

    def writer(self, message):
        self.message = message
        return message


def consumer(bus, dt):
    #takes control values
    done = 0
    px = Picarx()
    while done==0:
        steer_angle = bus.reader()
        px.set_dir_servo_angle(steer_angle)
        done = 1
    time.sleep(dt)



def producer(bus, dt):
    #produces actions
    done = 0
    while done == 0:
        cap=cv2.VideoCapture(0)
        ret, frame = cap.read()
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([30, 40, 0])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        cropped_edges = cv2.Canny(mask, 200, 400)
        rho = 1  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4)
        ls = np.array(line_segments)
        x = line_segments[:, 0]
        y = line_segments[:, 1]
        coeffs = np.polyfit(1,x,y)
        steering_angle = np.atan(-1*coeffs[1]/coeffs[0])*(np.pi/180)
        bus.writer(steering_angle)
        done = 1
    time.sleep(dt)

if __name__ == "__main__":

    bus = Bus(0) #intial angle = 0

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        eSensor = executor.submit(consumer, bus, 1)
        eInterpreter = executor.submit(producer, bus, 1)
