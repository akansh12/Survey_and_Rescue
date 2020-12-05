#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
# import imutils
import copy
import json

clm = ['A','B','C','D','E','F']
row = ['1','2','3','4','5','6']
A = []
# last_pub=[]


class sr_determine_colors():

	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
 		self.img = None
 		self.data = {}
 		self.last_pub = []

	def load_rois(self, file_path = 'rect_info.pkl'):
		with open("/home/default/catkin_ws/src/survey_and_rescue/scripts/RoIs.json", "r") as read_file:
			self.data = json.load(read_file)
			for i in clm:
			    for j in row:
			        A.extend([self.data[i+j]])

		# try:
			# s.rois = np.load("rois.npy")
			# with open("/home/default/catkin_ws/src/survey_and_rescue/RoIs.json", "r") as read_file:
			# 	self.data = json.load(read_file)
			# 	for i in clm:
			# 	    for j in row:
			# 	        A.extend([self.data[i+j]])
        



		# except IOError, ValueError:
		# 	print("File doesn't exist or is corrupted")


 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)


 	def serviced_callback(self, msg):
 		pass
 	
	def find_color_contour_centers(self):
		inf=[]
		j=0
		for i in A:

			loc=str(unichr(j//6+65))+str(j%6+1)
			j=j+1
			st=""
			x,y,w,h= i
			image = self.img
			cp_image=image[y:y+h,x:x+h] #------------------
			hsv = cv2.cvtColor(cp_image,cv2.COLOR_BGR2HSV)
			lower_red = np.array([140,105,210])
			upper_red = np.array([255,255,255])
			mask = cv2.inRange(hsv, lower_red, upper_red)

			# compute the center of the contour
			M = cv2.moments(mask)
			if M["m00"] <1000:
				pass
			else:
				# cX = int(M["m10"] / (M["m00"]))
				# cY = int(M["m01"] / (M["m00"]))
				


			# 	# draw the contour and center of the shape on the image
			# 	# cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				# cv2.circle(cp_image, (cX, cY), 7, (0, 0, 255), -1)
				# cv2.putText(cp_image, "red", (cX - 20, cY - 20),
				# 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				st="RESCUE"
				# print(loc)
				# print(M['m00'])
				inf.append((loc,st))
				continue



			# blue LED
			# hsv = cv2.cvtColor(cp_image,cv2.COLOR_BGR2HSV)
			lower_blue = np.array([100,150,150])
			upper_blue = np.array([130,255,255])
			mask = cv2.inRange(hsv, lower_blue, upper_blue)

			# compute the center of the contour
			M = cv2.moments(mask)
			if M["m00"] < 1000:
				pass
			else:
				# cX = int(M["m10"] / (M["m00"]))
				# cY = int(M["m01"] / (M["m00"]))


		# 	# draw the contour and center of the shape on the image
		# 	# cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				# cv2.circle(cp_image, (cX, cY), 7, (255, 0, 0), -1)
				# cv2.putText(cp_image, "blue", (cX - 20, cY - 20),
				# 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
				st="MEDICINE"
				# print(loc)

				# # print(loc)

				# print(M['m00'])
				inf.append((loc,st))
				continue


			# hsv = cv2.cvtColor(cp_image,cv2.COLOR_BGR2HSV)
			lower_green = np.array([52,25,212])
			upper_green = np.array([80,255,255])
			mask = cv2.inRange(hsv, lower_green, upper_green)

				# compute the center of the contour
			M = cv2.moments(mask)
			if M["m00"] < 1000:
				pass
			else:
				# cX = int(M["m10"] / (M["m00"]))
				# cY = int(M["m01"] / (M["m00"]))
			


		# 	# draw the contour and center of the shape on the image
		# 	# cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				# cv2.circle(cp_image, (cX, cY), 7, (0, 255, 0), -1)
				# cv2.putText(cp_image,'green', (cX - 20, cY - 20),
				# 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				st="FOOD"
				# print(loc)

				# print(M['m00'])
				inf.append((loc,st))
				continue

			

		for i in inf:
			flag = False
			for j in self.last_pub:
				if i==j:
					flag=True
					break
			if flag==False:
				self.detect_info_msg.location = i[0]
				self.detect_info_msg.info = i[1]
				self.detect_pub.publish(self.detect_info_msg)
				# print(i)
				#publish
		self.last_pub = inf


		# cv2.imshow('res',image)
		# cv2.waitKey(0)


		# cv2.waitKey(0)

				





	def check_whether_lit(self):
		pass


def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(30)
		# rate = rospy.Rate(5)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.find_color_contour_centers()
			s.check_whether_lit()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)