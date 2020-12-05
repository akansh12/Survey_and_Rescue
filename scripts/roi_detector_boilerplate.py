#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
# import imutils
import copy
import numpy as np
import itertools
import json

class sr_determine_rois():

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None
		self.cor =[]
		self.dic = {} 
		ls = []
	# CV_Bridge acts as the middle layer to convert images streamed on rostopics to a format that is compatible with OpenCV
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)


	'''You will need to implement an image processing algorithm to detect the Regions of Interest (RoIs)
	The standard process flow is:
	i)		Standard pre-processing on the input image (de-noising, smoothing etc.)
	ii)		Contour Detection
	iii)	Finding contours that are square, polygons with 4 vertices
	iv)		Implementing a threshold on their area so that only contours that are the size of the cells remain'''
	def detect_rois(self):
		# Add your Code here
		# You may add additional function parameters
		# cv2.imshow("Detected ROIs", img_copy) #A copy is chosen because self.img will be continuously changing due to the callback function
		# cv2.waitKey(100)
		# gray=cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
		# cv2.imshow("Gray Image",gray)
		# cv2.waitKey(0)

		# blured=cv2.GaussianBlur(self.img,(13,13),0)
		# # cv2.imshow("Gaussian blurred",blured)
		# # cv2.waitKey(0)

		# canny=cv2.Canny(blured,100,150)
		# # cv2.imshow("canny_edge_detection",canny)
		# # cv2.waitKey(0)


		# (_,cnts,_)=cv2.findContours(canny.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		# coins=self.img.copy()
		# cv2.drawContours(coins,cnts,-1,(255,255,255),2)
		# cv2.imshow('contours',coins)
		# cv2.waitKey(0)

		# imgray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
		# copy = self.img
		# imgray = cv2.GaussianBlur(copy,(65,65), 0)
		# sigma = 0.33
		# v = np.median(imgray)
		# low = int(max(0, (1.0 - sigma) * v))
		# high = int(min(255, (1.0 + sigma) * v))

#----------------------------------------------------------------------------------------------
		# ret,thresh = cv2.threshold(imgray,100,150,cv2.THRESH_BINARY_INV)
		# # th2 = cv2.adaptiveThreshold(imgray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
  # #           cv2.THRESH_BINARY,11,2)
		# img1, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		# cv2.imshow('Con',img1)
		# cv2.waitKey(0)
		# ls =[]

		# # cnt = contours[20]
		# # epsilon = 0.1*cv2.arcLength(cnt,True)
		# # approx = cv2.approxPolyDP(cnt,epsilon,True)

		# for c in contours:
		# 	peri = cv2.arcLength(c,True)
		# 	vertices = cv2.approxPolyDP(c,0.05*peri,True)
			
		# 	if len(vertices) == 4:
		# 		ls.append(c)
		# 		# image = cv2.drawContours(image, c, -1, (0,255,0), 3)
		# 		x, y, w, h = cv2.boundingRect(vertices)
		# 		cv2.rectangle(copy,(x,y),(x+w,y+h),(255,255,255),2)
		# 		cv2.imshow('Contour',copy)
				

		# cv2.waitKey(0)
#------------------------------------------------------------------------------------------------


		imgray1 = self.img
		# image = cv2.imread('test.jpg')
		# imgray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		imgray = cv2.GaussianBlur(imgray1,(17,17), 0)
		sigma = 0.33
		v = np.median(imgray)
		low = int(max(0, (1.0 - sigma) * v))
		high = int(min(255, (1.0 + sigma) * v))


		ret,thresh = cv2.threshold(imgray,low,high,cv2.THRESH_BINARY)
		img, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cv2.imshow('Con',img)
		cv2.waitKey(0)
		ls =[]

		# cnt = contours[20]
		# epsilon = 0.1*cv2.arcLength(cnt,True)
		# approx = cv2.approxPolyDP(cnt,epsilon,True)

		for c in contours:
			peri = cv2.arcLength(c,True)
			vertices = cv2.approxPolyDP(c,0.04*peri,True)
			a = cv2.contourArea(c)
			if len(vertices) == 4 and a>3000 and a< 6000:
				ls.append(c)
				# image = cv2.drawContours(image, c, -1, (0,255,0), 3)
				x, y, w, h = cv2.boundingRect(vertices)
				# cv2.rectangle(imgray1,(x,y),(x+w,y+h),(255,255,255),3)
				# cv2.imshow('Contour',imgray1)
				self.cor.append((x,y,w,h))
				# print(cv2.contourArea(c))
		cv2.waitKey(0)







		







	def sort_rois(self):
		# Add your Code here
		
		self.dic = {} 
		self.cor.sort()
		imgray1=self.img
		# print(self.cor)
		ls = []
		clm = ['A','B','C','D','E','F']
		row = ['1','2','3','4','5','6']
		for i in range(0,36,6):
			a = self.cor[i:i+6]
			a.sort(key = lambda x:x[1])
			ls.append(a)
			j=0
			for A in a:
				x,y,w,h = A
				cv2.rectangle(imgray1,(x,y),(x+w,y+h),(255,255,255),3)
				txt = clm[i/6]+row[j]
				self.dic.update({txt:A})
				# cv2.putText(image, txt,(x+w/2,y+h/2),cv2.FONT_HERSHEY_SIMPLEX,(255,255,255))
				cv2.putText(imgray1, txt, (x+w/4,y+h/2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), 2, cv2.LINE_AA)
				cv2.imshow('Contour', imgray1)

				cv2.waitKey(0)
				j+=1

		self.cor=ls

		



	def query_yes_no(self, question, default=None):
		"""Ask a yes/no question via raw_input() and return their answer.

		"question" is a string that is presented to the user.
		"default" is the presumed answer if the user just hits <Enter>.
		It must be "yes" (the default), "no" or None (meaning
		an answer is required of the user).

		The "answer" return value is True for "yes" or False for "no".
		"""
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:\t"
		elif default == "yes":
			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	'''	You may save the list using anymethod you desire
	 	The most starightforward way of doing so is staright away pickling the objects
		You could also save a dictionary as a json file, or, if you are using numpy you could use the np.save functionality
		Refer to the internet to find out more '''
	def save_rois(self):
		#Add your code here
		with open('RoIs.json', mode='w') as outfile:
			json.dump(self.dic, outfile)


	#You may optionally implement this to display the image as it is displayed in the Figure given in the Problem Statement
	def draw_cell_names(self, img):
		pass
		#Add your code here

def main(args):
	#Sample process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while True:
			if r.img is not None:
				r.detect_rois()
				if('''No of cells detected is not 36'''):
					r.sort_rois()
					r.save_rois()					
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to change ##Enter tweaks, this is not necessary##?")
					if(new_thresh_flag):
						pass
						#Change settings as per your desire
					else:
						continue
				else:
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if(satis_flag):
						r.sort_rois()
						r.save_rois()
						cv2.destroyAllWindows()
						break
					else:
						#Change more settings
						pass
		# r.draw_cell_names(r.img) # Again, this is optional
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)