
from picarx import Picarx
px = Picarx()
from ezblock import delay


def forever():
	while True:	
		val = input("Enter number:")
		if(int(val) == 1):
			px.forward(50)
			delay(1000)
		if(int(val) == 2):
			px.backward(50)
			delay(1000)
		if( int(val) == 3 ):
			px.set_dir_servo_angle((-30))
			delay(1000)
		if(int(val)==4):
			px.set_dir_servo_angle(30)
			delay(1000)
		px.stop()

if __name__ == "__main__":
    forever()