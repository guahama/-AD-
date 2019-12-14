import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetector:

    def __init__(self):
        self.image_width = 640
        self.scan_width, self.scan_height = 320, 40
        self.area_width, self.area_height = 20, 10
        area = self.area_width * self.area_height
	self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
	self.row_begin = (self.scan_height- self.area_height)//2
	self.row_end = self.row_begin + self.area_height
        self.pixel_cnt_threshold = area * 0.6
	self.linescan_offset = 15
        self.roi_vertical_pos = 260
        self.left, self.right = -1, -1
	

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber('/usb_cam/image_raw', Image, self.conv_image)

	
            

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]
	blur = cv2.GaussianBlur(roi, (5, 5), 0)
	test = cv2.Canny(blur, 50, 150)

	lines = cv2.HoughLinesP(test, 1, np.pi/180, 50, None, 50, 10)
    	if lines is not None:
        	for i in range(0, len(lines)):
        	    l = lines[i][0]
        	    cv2.line(test, (l[0], l[1]), (l[2], l[3]), (255, 255, 255), 3, cv2.LINE_AA)

        mid = cv2.cvtColor(test, cv2.COLOR_GRAY2BGR)
    	hsv = cv2.cvtColor(mid, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 250, 255], dtype=np.uint8)
        
	self.mask = cv2.inRange(hsv, lbound, ubound)
        self.view = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)

    def detect_lines(self):
	self.left, self.right = -1, -1
    	for l in range(self.lmid,self.area_width ,-1):
        	area = self.mask[self.row_begin:self.row_end, l - self.area_width:l] 
        	if cv2.countNonZero(area) > self.pixel_cnt_threshold:
        	    self.left = l
	            break
	    
	for r in range(self.rmid,self.image_width-self.area_width):
	        area = self.mask[self.row_begin:self.row_end, r:r + self.area_width]
	        if cv2.countNonZero(area) > self.pixel_cnt_threshold:
	            self.right = r
	            break
	
	
        # Return positions of left and right lines detected.
        return [self.left, self.right]

    def show_images(self, left, right):
	if left != -1:
		lsquare = cv2.rectangle(self.view,
					(left - self.area_width, self.row_begin),
                                	(left, self.row_end),
                                	(0, 255, 0), 3)
    	else:
        	print("Lost left line")

    	if right != -1:
        	rsquare = cv2.rectangle(self.view,
        	                        (right, self.row_begin),
        	                        (right + self.area_width, self.row_end),
        	                        (0, 0, 255), 3)
    	else:
        	print("Lost right line")
	
	cv2.imshow("view", self.view)
	cv2.waitKey(1)



class ColorDetector:

    def __init__(self):
        self.image_width = 640
        self.scan_width, self.scan_height = 120, 80
        # self.area_width, self.area_height = 20, 10
        area = 1700
	self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
	#self.row_begin = (self.scan_height- self.area_height)//2
	#self.row_end = self.row_begin + self.area_height
        self.pixel_cnt_threshold = area * 0.8
        self.roi_vertical_pos = 210
	self.color = ''
	

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber('/usb_cam/image_raw', Image, self.find_img)
    
    def find_img(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
	v = self.roi_vertical_pos
	roi = self.cam_img[v:v + self.scan_height, 40:160]
	self.hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
	
	cv2.imshow("k", roi)
	cv2.waitKey(1)

    def getColor(self):
	if self.detectRed():
	    self.color = 'red'
        elif self.detectBlue():
            self.color = 'blue'
        elif self.detectGreen():
            self.color = 'green'
        elif self.detectYellow():
            self.color = 'yellow'
       
        #elif self.detectPink():
            #self.color = 'pink'
        
        else:
            self.color = ''
	
    def detectRed(self):
	red_range = cv2.inRange(self.hsv,np.array([-10,90,90]),np.array([10,255,255]))
        if cv2.countNonZero(red_range) > self.pixel_cnt_threshold:
	    print("red <<<<<<<<<<<<<")
	    return True
	return False

    def detectBlue(self):
	blue_range = cv2.inRange(self.hsv,np.array([90,90,90]),np.array([110,255,255])) 
	if cv2.countNonZero(blue_range) > self.pixel_cnt_threshold:
	    print("blue <<<<<<<<<<")
	    return True
	return False

    def detectGreen(self):
	green_range = cv2.inRange(self.hsv,np.array([45,90,90]),np.array([65,255,255])) 
	if cv2.countNonZero(green_range) > self.pixel_cnt_threshold:
            print("green <<<<<<<<<<<<")
	    return True
	return False

    def detectYellow(self):
	yellow_range = cv2.inRange(self.hsv,np.array([20,90,90]),np.array([40,255,255])) 
	if cv2.countNonZero(yellow_range) > self.pixel_cnt_threshold:
	    print("yellow <<<<<<<<<<<")
	    return True
	return False

    #def detectPink(self):
	#pink_range = cv2.inRange(self.hsv,np.array([130,80,80]),np.array([165,255,255])) 
	#if cv2.countNonZero(pink_range) > self.pixel_cnt_threshold:
	    #print("pink <<<<<<<<<<<")
	    #return True
	#return False


if __name__ == '__main__':
	test = LineDetector()
	car = ColorDetector()
	time.sleep(3)
       	while not rospy.is_shutdown():
        	test.detect_lines()
		test.show_images(test.detect_lines()[0], test.detect_lines()[1])
		car.find_img()
	cv2.destroyAllWindows()

        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().

