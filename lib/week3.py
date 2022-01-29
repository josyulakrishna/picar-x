from concurrent.futures import thread
from cv2 import threshold
from adc import ADC
import numpy as np
from picarx import *

class Sensing(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref
    
    def calibrate(self):
        all_vals = []
        for i in range(100): 
            all_vals.append(self.get_grayscale_data())
        all_vals = np.array(all_vals)
        mean = all_vals.mean(axis=0)
        std = all_vals.std(axis=0)

        #assume gaussian noise and accept values within 1 std of the mean as that sensor value. 
        return (mean, std)
    
    #main sensing loop
    def get_status(self,fl_list):
        # roboustness check(can we rely on the data recieved), is the car slightly away from the mean or largely away 
        # take the 95% confidence interval calculate z-score
        polarity = fl_list[2]
        z = (self.ref-fl_list[0])/fl_list[1]
        if(z>2.0): 
            print("please keep the car aligned to the tape")
            return "stop"
        #threshold for value robustness = 5
        threshold = 3
        

        if fl_list[0] > self.ref  and fl_list[1] > self.ref and fl_list[2] > self.ref:
            return 'stop'
        elif (fl_list[1]-self.ref) <= threshold:
            print("system on tape status: 0 action: forward")
            return 0
        
        elif (fl_list[0]-self.ref) <= threshold:
            print("system on left of tape status: -1 action: right")
            return -1

        elif (fl_list[2]-self.ref) <=threshold :
            print("system on right of tape status: 1 action: left")
            return 1

    #main control loop
    #px = instance of picarx
    #dir = direction
    #percent = amount of battery power to apply
    def control(self, px, dir, pow):
         angle_step = 10
         if dir==0:
            print("forward")
            px.forward(pow)
            time.sleep(1)

         if dir==-1:
            print("right")
            #steer right with 10deg step
            time.sleep(1)
            px.set_dir_servo_angle(angle_step)
            time.sleep(1)

         if dir==1:
             print("left")
             #steer left with -10deg step
             time.sleep(1)
             px.set_dir_servo_angle(-1*angle_step)
             time.sleep(1)
         
    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list

if __name__ == "__main__":
    import time
    GM = Sensing(950)
    px = Picarx()
    (mean, std)=GM.calibrate()
    #follow tape, dark side
    polarity = 1
    # follow tape, bright one 
    # polarity = 0
    #assuming the A1(channel S1) as the reference signal we have calculated the mean and standard deviation of the data over 100 values 
    # to ensure roboustness we consider the values in range 1 standarad deviation from mean 
    while True:
        print(GM.get_grayscale_data())
        dir = GM.get_status([mean[1], std[1], polarity])
        #control takes direction and power
        GM.control(px, dir, 20)
        time.sleep(1)
