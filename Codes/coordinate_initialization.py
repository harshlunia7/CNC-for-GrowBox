
#****************************************
#Function Name:
#  * Input: 
#  * Output: 
#  * Logic: 
#  * Example Call: ()
#****************************************
import cv2
import numpy as np
import math
import serial
import os

#************Variables******************

global dist_y_bord
global dist_x_bord
global arduino_confirm
global comm_string
global ser
global line_in
global phase  # phase 1 : for red border and phase 2 : for black border
global distance
global found_red
global found_black
global mm_per_pix
global coord_ini_status

#************Initialization*************

ser = serial.Serial('/dev/ttyACM0',9600)
phase=1
found_red=False
found_black=False
dist_y_bord=20  #mm
dist_x_bord=20  #mm
mm_per_pix=0.364996557;
cap = cv2.VideoCapture(0)
cap.set(3,1280)
cap.set(4,1024)
time.sleep(2)
coord_ini_status=False

#**************************************
#************Functions******************

#****************************************
#Function Name:get_ratio_points
#  * Input: coordinates of two points and ratios
#  * Output: required coordinate
#			dividing line into req. ratio	
#  * Logic: line ratio formula
#  * Example Call: get_ratio_points(1,1,6,6,1,2)
#****************************************
def get_ratio_points(x1,x2,y1,y2,k1,k2):
  x0 = ((x1*k2)+(x2*k1))/(k1+k2)
  y0 = ((y1*k2)+(y2*k1))/(k1+k2)
  return int(x0),int(y0)

#****************************************
#Function Name:cal_distance_for_red
#  * Input: 4 coord. of border red 
#  * Output: none
#  * Logic: get center line of red border
#			cal. the distance from midpoint of photo 
#  * Example Call: cal_distance_for_red(1,2,3,4,5,6,7,8)
#****************************************
def cal_distance_for_red(x1,x2,x3,x4,y1,y2,y3,y4):
	xc,yc=(640,512)
	yr=512
	xt,yt=get_ratio_points(x1,x2,y1,y2,1,1)
	xd,yd=get_ratio_points(x2,x3,y2,y3,1,1)
	m=(yd-yt)/(xd-xt)
	c=yd-(m*xd)
	xr=(yr-c)/m;
	#distance in pixels
	distance = math.sqrt(math.pow(xc-xr,2)+math.pow(yc-yr,2))
	distance=distance*mm_per_pix

#****************************************
#Function Name:cal_distance_for_black
#  * Input: 4 coord. of border black 
#  * Output: none
#  * Logic: get center line of black border
#			cal. the distance from midpoint of photo 
#  * Example Call: cal_distance_for_black(1,2,3,4,5,6,7,8)
#****************************************
def cal_distance_for_black(x1,x2,x3,x4,y1,y2,y3,y4):
	xc,yc=(640,512)
	xr=640
	xt,yt=get_ratio_points(x1,x3,y1,y3,1,1)
	xd,yd=get_ratio_points(x2,x4,y2,y4,1,1)
	m=(yd-yt)/(xd-xt)
	c=yd-(m*xd)
	yr=m*xr+c
	#distance in pixels
	distance = math.sqrt(math.pow(xc-xr,2)+math.pow(yc-yr,2))
	distance=distance*mm_per_pix

#****************************************
#Function Name:go_for_y
#  * Input: int final-whether to go for final run
#  * Output: none
#  * Logic: will generate the serial string
# 			and communicate to arduino, followed
#			by confirmation  (for y direction)
#  * Example Call:go_for_y()
#****************************************
def go_for_y(int final):
	arduino_confirm=False
	mystring=""
	if final = 0 :
		comm_string="@"+"0000"+str(dist_y_bord)+"000"+"!"
	elif final =1 :
		comm_string="@"+"0000"+str(distance)+"000"+"!"
	ser.write(comm_string.encode("ascii"))
	while arduino_confirm is False :
		line_in=ser.readline()
		for char in line_in:
			mystring = mystring + chr(char)
		if mystring is "@done!" :
			arduino_confirm=True

#****************************************
#Function Name:go_for_x
#  * Input: int final-whether to go for final run
#  * Output: none
#  * Logic: will generate the serial string
# 			and communicate to arduino, followed
#			by confirmation(for x direction)  
#  * Example Call: go_for_x()
#****************************************

def go_for_x(int final):
	arduino_confirm=False
	mystring=""
	if final=0 :
		comm_string="@"+str(dist_x_bord)+"000"+"000"+"!"
	elif final=1 :
		comm_string="@"+str(distance)+"000"+"000"+"!"
	ser.write(comm_string.encode("ascii"))
	while arduino_confirm is False :
		line_in=ser.readline()
		for char in line_in:
			mystring = mystring + chr(char)
		if mystring is "@done!" :
			arduino_confirm=True

#****************************************
#Function Name:image_process
#  * Input: none
#  * Output: none 
#  * Logic: will take photo and process it 
#			to look for red border.
#			if found will call go_for_y
#			after changing dist_y_bord
#			as calculated
#  * Example Call: image_process_red()
#****************************************

def image_process():
	ret, img = cap.read()
	if cap.isOpened() :
		hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		if phase=1 :
			lower_range=np.array([160,50,50])
			higher_range=np.array([180,255,255])
		elif phase = 2:
			lower_range=np.array([,50,50])
			higher_range=np.array([,255,255])
		mask1 = cv2.inRange(hsv,lower_range,higher_range)
		res = cv2.bitwise_and(img,img, mask=mask1)
		ret,thrshed = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
		image,contours,hier = cv2.findContours(thrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		areas = [cv2.contourArea(c) for c in contours]
		max_index = np.argmax(areas)
		if areas[max_index]>13000 and phase=1 :
			found_red=True
		elif areas[max_index]>1000 and phase=2 :
			found_black=True
 		if found_red is True and phase=1 :
			cnt=contours[max_index]
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			x4,y4=box[0]
			x3,y3=box[1]
			x1,y1=box[2]
			x2,y2=box[3]
			cal_distance_for_red(x1,x2,x3,x4,y1,y2,y3,y4)
			go_for_y(1)
			phase = 2
		elif found_black is True and phase=2 :
			cnt=contours[max_index]
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			x4,y4=box[0]
			x3,y3=box[1]
			x1,y1=box[2]
			x2,y2=box[3]
			cal_distance_for_black(x1,x2,x3,x4,y1,y2,y3,y4)
			go_for_x(1)
#****************************************
#Function Name:coord_ini
#  * Input: none
#  * Output: none
#  * Logic: will go for coordinate initialisation
#  * Example Call: coord_ini()
#****************************************
def coord_ini() :
	while found_red is False and found_black is False :
		if phase=1:
			go_for_y(0)
			image_process()
		elif phase=2:
			go_for_x(0)
			image_process()
	coord_ini_status=True
	cap.release()



