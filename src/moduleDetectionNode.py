#!/usr/bin/env python


# Author       : Yaakoubi Oussama
# Organism     : Amac-Isir (In the scoop of the Dream project)
# Description  : This node is used to detect modules(Lever, joystick, button) on an interface, 
#               in order to provide a robot a feedback about what he interacted with during 
#               an experiment. 
import sys
import freenect
import cv2
import numpy as np
import roslib
import rospy
from std_msgs.msg import String
from dream_demos_setup.msg import Joystick, State

class ModuleDetection(): 
		
	def isset(self,v): 
		try: 
			type (eval(v)) 
		except: 
			return 0 
		else: 
			return 1

	def getKinectVideoFeed(self):
		array,_ = freenect.sync_get_video()
		array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
		return array

	def locateAreaOfIntrest(self, hsvFrame, lowerBound, upperBound):
		# Filter the area of intrest to get modules color tag 
		threshInterface = cv2.inRange(hsvFrame,lowerBound, upperBound)
		#threshInterface2 = threshInterface.copy()

		# Find modules in the filtered image
		contours,_ = cv2.findContours(threshInterface, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		# For each contour id the module depending on its relative position to the center of the interface
		maxArea = 0
		best_fit = None
		if(len(contours) == 0) :
			print("no contour detected")
			sys.exit(1)

		for cnt in contours:
			area = cv2.contourArea(cnt)
			if area > maxArea:
				maxArea = area
				best_fit = cnt

		if(not best_fit) :
			print("no best fit contour")
			sys.exit(1)
		# Return coordinates and dimentions of the detected module

		moduleCoodDim = cv2.boundingRect(best_fit)
			# epsilon = 0.1*cv2.arcLength(best_fit,True)
			# approx = cv2.approxPolyDP(best_fit,epsilon,True)
		
		return moduleCoodDim

	def getButtonCurrentState(self, frame, hsvFrame):
		moduleName = "Button"
		moduleState = ""
		stateDico = {'name':moduleName,'state':moduleState}

		# Specify color range for module detection
		colorRangeLowerBound = np.array((120, 80, 80))
		colorRangeUpperBound = np.array((180, 255, 255))    


		# Get module's coordinates and dimentions within the frame
		x,y,w,h = self.locateAreaOfIntrest(hsvFrame, colorRangeLowerBound, colorRangeUpperBound)
		if (w == 0):
			rospy.logwarn("The Best fit for the button is no longer detected :( ...")
		else:
			# Retrieve module  from the original frame
			button = frame[y:y+h,x:x+w] 
			hsvButton = hsvFrame[y:y+h,x:x+w] 
			# Specify color range for module detection
			colorRangeLowerBound = np.array((50, 80, 80))
			colorRangeUpperBound =  np.array((120, 255, 255))
			# Get lever's position
			xB,yB,wB,hB = self.locateAreaOfIntrest(hsvButton, colorRangeLowerBound, colorRangeUpperBound) 
			# If a button is on, the center od the detected color will be relatively close to half of the width of the module
			if ((xB+wB/2-w/2) < 2) & ((yB+hB/2-h/2) < 2):
				moduleState = 'ON'
			else:
				moduleState = 'OFF'  

			# Draw a rectangle around the detected module
			cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,0),2)
			cv2.putText(frame,moduleName,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)       
			cv2.putText(frame,moduleState,(x+w/2,y+h/2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1, cv2.LINE_AA)
			stateDico = {'name':moduleName,'state':moduleState}
		return stateDico  

	def getLeverCurrentState(self, frame, hsvFrame):
		moduleName = "Lever"
		moduleState = ""
		stateDico = {'name':moduleName,'state':moduleState}
		# Specify color range for module detection
		colorRangeLowerBound = np.array((120, 80, 80))
		colorRangeUpperBound = np.array((180, 255, 255))    


		# Get module's coordinates and dimentions within the frame
		x,y,w,h = self.locateAreaOfIntrest(hsvFrame, colorRangeLowerBound, colorRangeUpperBound)
		if (w == 0):
			rospy.logwarn("The Best fit for the Lever is no longer detected :( ...")
		else:
			
			# Retrieve module  from the original frame
			lever = frame[y:y+h,x:x+w] 
			hsvLever = hsvFrame[y:y+h,x:x+w]  
				   
			# Specify color range for module detection
			colorRangeLowerBound = np.array((50, 80, 80))
			colorRangeUpperBound =  np.array((120, 255, 255))
			# Get lever's position
			xL,yL,wL,hL = self.locateAreaOfIntrest(hsvLever, colorRangeLowerBound, colorRangeUpperBound)    
			if (wL == 0):
				rospy.logwarn("The Best fit for the Lever is no longer detected :( ...")
			else:				
				maxLever = xL+wL
				if maxLever < w/4:
					moduleState = "gear0"
				elif (maxLever > w/4) & (maxLever < w/2):
					moduleState = "gear1"
				elif (maxLever > w/2) & (xL < w/2):
					moduleState = "gear2"
				elif (maxLever > w/2) & (xL < 3*w/4):
					moduleState = "gear3" 
				elif (maxLever > 3*w/4):
					moduleState = "gear4" 
				else:
				    moduleState = "unknown"
				stateDico['state'] = moduleState
				
				cv2.rectangle(lever,(xL,yL),(xL+wL,yL+hL),(0,0,255),2)
				cv2.putText(frame,moduleState,(x,y+h/2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1, cv2.LINE_AA)

			# Draw a rectangle around the detected module
			cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,0),2)
			cv2.putText(frame,moduleName,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

		return stateDico        

	def getJoystickCurrentState(self, frame, hsvFrame):
		moduleName = "Joystick"
		moduleState = []
		stateDico = {'name':moduleName,'state':moduleState}
		# Specify color range for module detection
		colorRangeLowerBound = np.array((120, 80, 80))
		colorRangeUpperBound = np.array((180, 255, 255))

		# Get module's coordinates and dimentions within the frame
		x,y,w,h = self.locateAreaOfIntrest(hsvFrame, colorRangeLowerBound, colorRangeUpperBound)
		if (w == 0):
			rospy.logwarn("The Best fit for the Joystick is no longer detected :( ...")
		else:
			joystick = frame[y:y+h,x:x+w]
			hsvJoystick = hsvFrame[y:y+h,x:x+w]
			# Specify color range for module detection
			colorRangeLowerBound = np.array((50, 80, 80))
			colorRangeUpperBound =  np.array((120, 255, 255))
			# Get joystick's center coordinates
			xJ,yJ,wJ,hJ = self.locateAreaOfIntrest(hsvJoystick, colorRangeLowerBound, colorRangeUpperBound)
			if (wJ == 0):
				rospy.logwarn("The Best fit for the Joystick's center is no longer detected :( ...")
			else:
				# Convert Joystick position to oXY frame [-1 1]
				newX = 2*(xJ*1.0+wJ*1.0/2-w*1.0/2)/w
				newY = 2*(yJ*1.0+hJ*1.0/2-h*1.0/2)/h  
				stateDico['state'] = [newX, newY]

				moduleState = "x=%.2f, y=%.2f"%(newX,newY)
				cv2.rectangle(joystick,(xJ,yJ),(xJ+wJ,yJ+hJ),(0,0,255),2)
				cv2.putText(frame,moduleState,(x,y+h/2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1, cv2.LINE_AA)

			# Draw a rectangle around the detected module
			cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,0),2)
			cv2.putText(frame,moduleName,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

		return stateDico
		
	def runDetection(self):
		rospy.init_node('moduleDetectionNode', anonymous=True)
		pubLever = rospy.Publisher('lever', State, queue_size = 10)	
		pubButton = rospy.Publisher('button', State, queue_size = 10)
		pubJoystick = rospy.Publisher('joystick', Joystick, queue_size = 10)
		
		rospy.loginfo("Launching detection")
		rospy.loginfo("Press Esc to quit")

		while(1):
			# Read the frames frome a camera
			frame = self.getKinectVideoFeed()
			frame = cv2.blur(frame,(3,3))

			# Convert the image to hsv space and find range of colors
			hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			
			# Filter teh frame to get the interface color tag
			thresh = cv2.inRange(hsvFrame,np.array((50, 80, 80)), np.array((120, 255, 255)))

			thresh2 = thresh.copy()

			# Find contours in the filtered image
			contours,_ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

			# Test for the contour with maximum area and store it as best fit for the interface
			max_area = 0
			for cnt in contours:
				area = cv2.contourArea(cnt)
				if area > max_area:
					max_area = area
					best_fit = cnt

			# Compute the center of the detected interface
			cx, cy = 0,0    # coordinates of the center of the detected interface
			x, y = 0,0      # coordinates of the detected interface
			h,w = 0,0       # height and width of the detected interface
			if isset('best_fit'):
				M = cv2.moments(best_fit)
				x,y,w,h = cv2.boundingRect(best_fit)
			if w == 0:
				rospy.loginfo("WARNING: The Best fit for the Interface is no longer detected :( ...")
			else:
				cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
				cv2.circle(frame,(cx,cy),5,255,-1)
				cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
				#print "Central pos: (%d, %d)" % (cx,cy)
				
				# Retrieve area of intrest  from the original frame
				areaOfIntrest = frame[y:y+h,x:x+w]

				# Retrieve area of intrest of each module
				leverAreaOfIntrest = areaOfIntrest[0:h/2,0:w]
				joystickAreaOfIntrest = areaOfIntrest[h/2:h,0:w/2]
				buttonAreaOfIntrest = areaOfIntrest[h/2:h,w/2:w]

				# Retrieve area of intrest from hsv frame for further processing
				hsvAreaOfIntrest = hsvFrame[y:y+h,x:x+w]

				# Retrieve area of intrest of each module from hsv frame 
				hsvLeverAreaOfIntrest = hsvAreaOfIntrest[0:h/2,0:w]
				hsvJoystickAreaOfIntrest = hsvAreaOfIntrest[h/2:h,0:w/2]
				hsvButtonAreaOfIntrest = hsvAreaOfIntrest[h/2:h,w/2:w]
				
				# Retrieve the current state of each module
				leverInfo = self.getLeverCurrentState(leverAreaOfIntrest, hsvLeverAreaOfIntrest)
				joystickInfo = self.getJoystickCurrentState(joystickAreaOfIntrest, hsvJoystickAreaOfIntrest)
				buttonInfo = self.getButtonCurrentState(buttonAreaOfIntrest, hsvButtonAreaOfIntrest)
				
				# Publish module state to the related topics
				leverMsg = State()
				leverMsg.name = leverInfo['name']
				leverMsg.state = leverInfo['state']
				pubLever.publish(leverMsg)
				
				buttonMsg = State()
				buttonMsg.name = buttonInfo['name']
				buttonMsg.state = buttonInfo['state']
				pubButton.publish(buttonMsg)
								
				joystickMsg = Joystick()
				joystickMsg.name = joystickInfo['name']
				joystickMsg.position = joystickInfo['state']
				pubJoystick.publish(joystickMsg)
							
				cv2.imshow('Area Of Intrest', areaOfIntrest)

			# Show the original and processed feed 
			cv2.imshow('Original feed', frame)
			cv2.imshow('thresh', thresh2)
			#cv2.imshow('threshInterface', threshInterface2)


			# if 'Esc' is pressed then exit the loop
			if cv2.waitKey(33)== 27:
				break
			
		# Destroy all windows exit the program
		cv2.destroyAllWindows()
		cap.release()
		
if __name__ == "__main__":
	detect = ModuleDetection() 
	detect.runDetection()
