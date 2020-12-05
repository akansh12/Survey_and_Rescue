#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import json
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from geometry_msgs.msg import PoseArray
import time
import csv
with open("/home/default/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json", "r") as read_file:
	data = json.load(read_file)


class sr_scheduler():

	def __init__(self):
		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		rospy.Subscriber('/stats_sr',SRDroneStats,self.stats_callback)
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		# self.stats_msg = SRDroneStats()

		self.foodOnboard = 0 # stores the food Onboard the drone form stat_sr topic.
		self.medOnboard = 0 # stores the medicine Onboard the drone form stat_sr topic.
		self.decided_msg = SRInfo()
		self.serviced_msg = SRInfo()
		self.red_lit = {} # stores the current red lit light's location in the dictionary.
		self.green_lit={} # stores the current green lit light's location in the dictionary.
		self.blue_lit={} # stores the current blue lit light's location in the dictionary.
		self.base =''
		self.read_tsv()
		self.drone_position = data[self.base] # stores the current drone position from '/whycon/poses' topic.
		self.decision_queue = [] # queue containing the coordinates to be hovered by drone.
		self.flag = 0 # flag variable used to tell that drone is hovering over base till the resources are replenished.
		# self.red_success=0
		self.prev_dec =''


# ```

# * Function Name: stats_callback
# * Input: msg
# * Output: none
# * Logic: used by '/stats_sr' topic to subscribe foodOnboard and medsOnboard data
# *
# * Example Call: NONE

# ```
	def stats_callback(self,msg):
		self.foodOnboard = msg.foodOnboard
		self.medOnboard = msg.medOnboard
		#store the food and medicine onboard the drone from /stats_sr topic.

		

# ```

# * Function Name:whycon_callback
# * Input: msg
# * Output: None
# * Logic: subscribe the 'whycon/poses' topic and stores the drone position in self.drone_position variable
# *
# * Example Call: NONE

# ```


	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#store the drone position(coordinates) from '/whycon/poses' topic
		
# ```

# * Function Name: detection_callback
# * Input: msg
# * Output: NONE
# * Logic: stores the current lit leds in their respective dictionarys
# *if red led is detected it's coordinate is stored in self.red_lit; if green light is detected, it's coordinate is stored in self.green_lit and so on.
# * Example Call: NONE

# ```


	def detection_callback(self, msg):
		self.decided_msg.location = msg.location
		self.decided_msg.info = msg.info

		#if red led is lit, store in self.red_lit.
		if msg.info == "RESCUE": 
			self.red_lit[self.decided_msg.location]=self.decided_msg.info
		

		#if blue led is lit, store in self.blue_lit.
		elif msg.info == "MEDICINE": 
			self.blue_lit[self.decided_msg.location]=self.decided_msg.info
		

		#if green led is lit, store in self.green_lit.
		else: 
			self.green_lit[self.decided_msg.location]=self.decided_msg.info




# ```

# * Function Name: serviced_callback
# * Input: msg
# * Output: NONE
# * Logic: when a led is turned off, it is removed from its respective dictionary.
# *
# * Example Call: called when a led is turned off

# ```


	def serviced_callback(self,msg):
		# Take appropriate action when either service SUCCESS or FAILIURE is recieved from monitor.pyc
		self.serviced_msg.location = msg.location
		self.serviced_msg.info = msg.info


		if self.serviced_msg.info == 'END':
			self.decision_queue.append(self.base) # append base in decision queue
			
			self.decided_msg.location = self.base
			self.decided_msg.info = 'BASE'
			self.decision_pub.publish(self.decided_msg) # publish BASE in '/decision_info' topic.
			self.flag=1

		elif msg.location in self.decision_queue: #if the led turned off is the led over which drone is hovering, then remove that coordinate from self.decision_queue.
			self.decision_queue.remove(msg.location)
			
			#if drone has hovered over red led for 5 sec, command the drone to hover over BASE.
			if (msg.location in self.red_lit) and msg.info == 'SUCCESS': 
				print('BASE')
				self.decision_queue.append(self.base) # append base in decision queue
				
				self.decided_msg.location = self.base
				self.decided_msg.info = 'BASE'
				self.decision_pub.publish(self.decided_msg) # publish BASE in '/decision_info' topic.

				self.flag=1 #base flag as 1 to denote that drone is hovering over base.
				


		if msg.location in self.red_lit: # if led turned off is red, then remove it from self.red_lit dictionary.
			del self.red_lit[self.serviced_msg.location]
			
		if msg.location in self.blue_lit: # if led turned off is blue, then remove it from self.blue_lit dictionary.
			del self.blue_lit[self.serviced_msg.location]

		if msg.location in self.green_lit: # if led turned off is green, then remove it from self.green_lit dictionary.
			# try:
			del self.green_lit[self.serviced_msg.location]
			# except:
			# 	print('not removed')




# ```

# * Function Name: dec_info
# * Input: st= tuple storing closest location and its distance from drone; lit is dictionary of lit lights(st from check function and lit is class variable)
# * Output: publishes the decision made 
# * Logic: it is called when some decision is need to be published
# *
# * Example Call: self.dec_info(st,self.blue_lit)

# ```

	def dec_info(self,st,lit):
		self.decided_msg.location = st[0]
		self.decided_msg.info = lit[st[0]]
		self.decision_queue = []	#emptying list
		self.decision_queue.append(st[0])
		self.prev_dec = self.decided_msg.info
		self.decision_pub.publish(self.decided_msg)




	def shutdown_hook(self):
		# This function will run when the ros shutdown request is recieved.
		# For instance, when you press Ctrl+C when this is running
		# self.disarm()

		pass

# ```
# * Function Name:read_tsv
# * Input: NONE
# * Output: NONE
# * Logic: read tsv file to find 
# *
# * Example Call: self.select_red()
# ```

	def read_tsv(self):
		with open("/home/default/catkin_ws/src/survey_and_rescue/scripts/LED_Bonus_Config.tsv") as tsvfile:
			tsvreader = csv.reader(tsvfile, delimiter="\t")
			for line in tsvreader:
				if line[-1] == 'BASE':
					self.base=line[0]



# ```
# * Function Name:select_red
# * Input: NONE
# * Output: NONE
# * Logic: of all the red leds lit, select the nearest one and publish it in /decision_info
# *
# * Example Call: self.select_red()
# ```

	def select_red(self): #if rescue
		st = self.check(self.red_lit) # returns the tuple containing nearest red led coordnate and its distance.
		if st[0] != '': # if any red led is lit, it will not be empty
			self.dec_info(st,self.red_lit)


# ```
# * Function Name:select_blue
# * Input: NONE
# * Output: NONE
# * Logic: of all the blue leds lit, select the nearest one and publish it in /decision_info
# *
# * Example Call: self.select_blue()
# ```

	def select_blue(self): #if MEDICINE
		st = self.check(self.blue_lit) # returns the tuple containing nearest blue led coordnate and its distance.
		if st[0] != '': # if any blue led is lit, it will not be empty
			self.dec_info(st,self.blue_lit)


# ```
# * Function Name:select_green
# * Input: NONE
# * Output: NONE
# * Logic: of all the green leds lit, select the nearest one and publish it in /decision_info
# *
# * Example Call: self.select_green()
# ```

	def select_green(self): #if FOOD
		st = self.check(self.green_lit) # returns the tuple containing nearest green led coordnate and its distance.
		if st[0] != '': # if any green led is lit, it will not be empty
			self.dec_info(st,self.green_lit)


	




# ```

# * Function Name: selector
# * Input: NONE
# * Output: NONE

# * Logic: if any red led is lit, the decision_info is published for the closest one otherwise decision_info is published for the other nearest led.
# * this function is called in scheduler function when drone is not hovering over any lit led.

# * Example Call: elif len(self.decision_queue) == 0: #self.selector()
# ```

	def selector(self):

		if not self.red_lit: # if not rescue 
			time.sleep(0.100)
			if (self.foodOnboard==0 and len(self.blue_lit)==0) or (self.medOnboard==0 and len(self.green_lit)==0):
				#goto base
				if self.flag==0:
					self.decided_msg.location = self.base
					self.decided_msg.info = 'BASE'
					self.decision_pub.publish(self.decided_msg)
					self.flag=1


			elif self.foodOnboard == 0:
				self.flag=0
				self.select_blue()

			elif self.medOnboard == 0:
				self.flag=0
				self.select_green()
			else:
				self.flag=0
				st1 = self.check(self.green_lit)
				st2 = self.check(self.blue_lit)

				if st1[0] !='' or st2[0] !='':

					if st1[0]=='':
						print('a')
						self.dec_info(st2,self.blue_lit)
					elif st2[0]=='':
						print('b')
						self.dec_info(st1, self.green_lit)
					elif st1[1]>st2[1]:
						print('c')
						self.dec_info(st2, self.blue_lit)
					else:
						print('d')
						self.dec_info(st1, self.green_lit)


				
		else: #if rescue
			self.select_red()




# ```

# * Function Name: check
# * Input: lit dictionary (can be red_lit, green_lit, blue_lit)
# * Output: returns the tuple containing nearest location coordinate and its distance
# * Logic: from the given lit dictionary, calculate the distance of all leds in the list and returns the nearest one.
# *
# * Example Call: st1 = self.check(self.green_lit)

# ```

	def check(self, lit): #check for min distance
		sm_dis=('', 9999999.999) #(RoI, distance)
		for elem in lit:
			led = data[elem]
			dis = ((led[0]-self.drone_position[0])**2 + (led[1]-self.drone_position[1])**2)**0.5
			
			if dis<sm_dis[1]:
				sm_dis=(elem, dis)
		
		return sm_dis



# ```

# * Function Name: scheduler
# * Input: NONE
# * Output: NONE
# * Logic: main function calling scheduling the hover decision. It contains three if-else condition; first one (if) is true for zero resource.
# * second one (elif) is true when decision_queue is empty and drone is not hovering over any lit led.
# * third one (else) is true when drone is hovering over some lit led or base i.e. decision_queue is not empty.

# * Example Call: called continuously in main function

# ```

	def scheduler(self):

		print(self.decision_queue)
		# print("red")
		print(self.red_lit)
		# print("green")
		print(self.green_lit)
		# print("blue")
		print(self.blue_lit)
		# print(type(self.foodOnboard))
		if self.foodOnboard == 0 and self.medOnboard == 0: # zero resources
			#goto base
			# print('a')
			# time.sleep(0.150)
			if self.flag == 0:
				self.decided_msg.location = self.base
				self.decided_msg.info = 'BASE'
				# print('base2')
				self.decision_pub.publish(self.decided_msg)
				self.flag=1
			# time.sleep(0.300)
			if ( not(self.base in self.decision_queue)) and len(self.decision_queue) == 0 and self.prev_dec != 'RESCUE': # continue searching red
				# print('a')
				self.select_red()


		elif len(self.decision_queue) == 0: # continue searching
			# print('b')
			self.selector()


		else: # dequeue BASE
			if (self.decision_queue[0] in self.blue_lit) or (self.decision_queue[0] in self.green_lit):
				# print('a')
				self.select_red()

			else:
				try:
					time.sleep(0.100)
					# self.red_success=0
					if self.foodOnboard != 0 or self.medOnboard != 0:
						self.decision_queue.remove(self.base)
						print(self.decision_queue)
						self.flag = 0
				except:
					pass



def main(args):
	
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)
	rate = rospy.Rate(30)
	sched.read_tsv()
	while not rospy.is_shutdown():
		sched.scheduler()
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)