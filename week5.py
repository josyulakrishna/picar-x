from RossROS.rossros import *
import numpy as np
import cv2
from lib.picarx import Picarx

from ezblock import *


time_bus = Bus(True, "time_bus")
timer = Timer(timer_busses=time_bus)
steer_bus = Bus(0, "steer_bus")
term_bus_s = Bus(0, "term_bus")
sonar_bus = Bus(100,"sonar_bus")
dt = 1

def consumer():
    #takes control values
    #argv[0] = steer_bus
    #argv[1] = time_bus
    px = Picarx()
    vals = consumer_stAngle.collectBussesToValues((time_bus,steer_bus))
    print("consumer", vals)
    while timer.timer():
        steer_angle = steer_bus.get_message()
        px.set_dir_servo_angle(steer_angle)
        print("steering _angle")



def producer():
    #produces actions
    producer_stAngle.dealValuesToBusses(False, term_bus_s)
    vals = producer_stAngle.collectBussesToValues((time_bus, steer_bus))
    print("producer", timer.timer())
    print(time_bus.get_message("producer"))
    while timer.timer():
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
        steering_angle = 0
        producer_stAngle.dealValuesToBusses((steering_angle, term_bus_s), (steer_bus, True))

def consumer_sonar():
    #Ultra Sonic Control
    dist=consumer_sonar.collectBussesToValues(sonar_bus)
    px = Picarx()
    if dist<=10:
        px.forward(0)

def producer_sonar():
    #Ultra Sonic
    distance = None
    pin_D0 = Pin("D0")
    pin_D1 = Pin("D1")
    distance = Ultrasonic(pin_D0, pin_D1).read()
    print("%s" % distance)
    producer_sonar.dealValuesToBusses((distance),(sonar_bus))


producer_stAngle = Producer(producer_function=producer, output_busses=(steer_bus, term_bus_s), delay=dt, name="sense")
consumer_stAngle = Consumer(consumer_function=consumer, input_busses=(steer_bus,time_bus), delay=dt, name="control")
producer_sonar = Producer(producer_function=producer_sonar, output_busses=(sonar_bus, time_bus, term_bus_s), delay=dt, name="sense_sonar")
consumer_sonar = Consumer(consumer_function=consumer_sonar, input_busses=(sonar_bus, time_bus), delay=dt, name="control_sonar")

if __name__ == "__main__":
    # intial angle = 0
    runConcurrently([producer_stAngle, consumer_stAngle])

