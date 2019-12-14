#!/usr/bin/env python

import rospy, rospkg, time, cv2, numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import Int32MultiArray
from findAD import LineDetector, ColorDetector
from imuread import ImuRead

class AutoDrive:


    def __init__(self):
        rospy.init_node('drive')
        self.pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
        self.position = LineDetector()
	self.color = ColorDetector()
        self.lPos = -1
        self.rPos = -1
        self.speed = 128
	self.imu = ImuRead('/diagnostics')


    def exit(self):
        print("finished")

    def auto_drive(self, Angle, Speed):
        drive_info = [Angle, Speed]
        drive_info = Int32MultiArray(data=drive_info)
        self.pub.publish(drive_info)

    def trace(self):
        offset = 10
	self.pos = self.position.detect_lines()
	self.lPos, self.rPos = self.pos[0], self.pos[1]
	#self.position.show_images(self.lPos, self.rPos)

	
	if self.lPos > 320:
                self.lPos = -1
        if self.rPos < 320:
                self.rPos = -1

	if abs(self.lPos-self.rPos) <= 80:
	    if self.rPos >= 380:
		x_location = 90-(self.rPos-400-offset)*0.6
	   	if x_location < 40:
		    x_location = 40
   	        self.auto_drive(x_location, self.speed)
	    elif self.lPos <= 280:
		x_location = 90+(240-self.lPos-offset)*0.6
		if x_location > 140:
		    x_location = 140
		self.auto_drive(x_location, self.speed)

	if self.lPos == -1:
	    x_location = (self.rPos-320-offset)*0.6
	    if x_location < 40:
		x_location = 40
            self.auto_drive(x_location, self.speed)
        elif self.rPos == -1:
	    x_location = (320-self.lPos-offset)*0.6
	    if x_location < 40:
		x_location = 40
            self.auto_drive(180-x_location, self.speed)

        elif self.lPos != -1 and self.rPos != -1:
            midPos = (self.lPos + self.rPos) // 2

            if (midPos >= 320):
                x_location = 90+(midPos-320)*0.3
            elif (midPos < 320):
                x_location = 90-(320-midPos)*0.3

            if x_location < 40:
                x_location = 40
	    elif x_location > 140:
		x_location = 140

            self.auto_drive(x_location, self.speed)
	
	elif self.lPos == -1 and self.rPos == -1:
	    self.auto_drive(90, 90)
	
    def pitch(self):
	r, p, y = self.imu.get_data()
	print('R (%.1f) P (%.1f), Y (%.1f)' % (r, p, y))	

	if abs(p) > 4:
	    return True
    
    def colorDetect(self):
	if self.color.color == 'red':      # stop
	    self.auto_drive(90, 90)
	    time.sleep(3)
	    self.auto_drive(90, 115)
	    time.sleep(2)
	    self.speed = 125

	elif self.color.color == 'yellow': # slow drive
	    for i in range(30):
		self.speed = 115
		self.trace()
	    self.speed = 128

	#elif self.color.color == 'pink':   # load
	    #pass
	
	elif self.color.color == 'green':  # unload 
	    for i in range(40):
	        self.auto_drive(120, 115)
	        time.sleep(0.1)
	    self.auto_drive(90, 90)
	    time.sleep(5)
	
	elif self.color.color == 'blue':   # oil
	    for i in range(35):
	        self.auto_drive(125, 115)
	        time.sleep(0.1)
	    for i in range(15): 
	        self.auto_drive(60, 115)
	        time.sleep(0.1)
	    time.sleep(10)
	    for i in range(35):
	    	self.auto_drive(60, 115)
	    	time.sleep(0.1)
	    for i in range(20):
	    	self.auto_drive(125, 115)
	    	time.sleep(0.1)
	    self.trace()

    #def drive_outside(self):
	#for i in range(15):
	    #self.auto_drive(110, 115)
	    #time.sleep(0.1)
	#for i in range(15): 
	    #self.auto_drive(70, 115)
	    #time.sleep(0.1)

    #def drive_inside(self):
	#for i in range(15):
	    #self.auto_drive(70, 115)
	    #time.sleep(0.1)
	#for i in range(15):
	    #self.auto_drive(110, 115)
	    #time.sleep(0.1)
	

if __name__ == "__main__":
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    for i in range(2):
        car.auto_drive(90, 90)
        time.sleep(0.1)
        car.auto_drive(90,70)
        time.sleep(0.1)
    time.sleep(5)
    
    while not rospy.is_shutdown():
	car.color.getColor()
	if car.color.color != '':
	    car.colorDetect()
	    time.sleep(0.1)
	else:
	    if car.pitch():
                car.auto_drive(90, 115)
		time.sleep(0.1)
            if not car.pitch():
                car.trace()
		time.sleep(0.1)
	
	car.color.color == ''


    rospy.on_shutdown(car.exit())

