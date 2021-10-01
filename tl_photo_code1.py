# -*- coding: utf-8 -*-
"""

@author: Chaitanya Lal
"""
# importing libraries.
import cv2
import numpy as np
import winsound

# Defining frequency and duration of warning sound.
frequency = 2500
duration = 50

# Vehicle's speed.
v= 50
# Input Frame.
cap = cv2.imread(r'C:\Users\91742\Desktop\Capstone project\sample_image4.png')

# Resizing and ROI selection.
img = cv2.resize(cap, (576,324))
roi = img[150:550, 150:550]

# Converting frame from BGR to HSV. Red and White Colour extraction.
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
lower_red1 = np.array([0,0,200])
upper_red1 = np.array([15,77,255])
lower_red2 = np.array([169,0,200])
upper_red2 = np.array([179,77,255])
lower_white=np.array([0,10,245])
upper_white=np.array([179,50,255])
mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask3=mask1+mask2
mask4 = cv2.inRange(hsv, lower_white, upper_white)

# Morphological Operation.
kernel1 = np.ones((5,5),np.uint8)
kernel2 = np.ones((2,2),np.uint8)
closing_red = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, kernel1)
closing_white = closing_red = cv2.morphologyEx(mask4, cv2.MORPH_CLOSE, kernel1)
res = cv2.bitwise_and(closing_red, closing_white)
opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel2)

# Generating contours over marked region.
temp2,contours,temp1 = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

# Functions of calculating symmetry score.

DS = lambda  Ha,Hb : (1 - (abs(Ha-Hb)/(Ha+Hb)))*100
def AS(Wa,Ha,Wb,Hb):
	Aa,Ab= Wa*Ha,Wb*Hb
	return DS(Aa,Ab)

"""def AR(data1,data2):
	c1,c2 = centre(data1),centre(data2)
	return abs(c1[0] - c2[0])/abs(c1[1] - c2[1])"""
AR = lambda data : data[2]/data[3]
def ARS(data1,data2):
	AR1,AR2 = AR(data1),AR(data2)
	return DS(AR1,AR2)



OppCor = lambda data: (data[0]+data[2],data[1]+data[3])
def CRegion(data1,data2):
	OC1,OC2 = OppCor(data1),OppCor(data2)
	cor1 = (min(data1[0],data2[0]),min(data1[1],data2[1]))
	cor2 = (max(OC1[0],OC2[0]),max(OC1[1],OC2[1]))
	return [cor1,cor2]

# Visualize symmetry score on output window.
def visualizer(x,t):
	retArr = []
	for idx ,cnt1 in enumerate(x) :
		MD = cv2.boundingRect(cnt1)
		for nxtIdx,cnt2 in enumerate(x[idx+1: ]):
			TD = cv2.boundingRect(cnt2)
			scores = [DS(MD[3],TD[3]),AS(MD[2],MD[3],TD[2],TD[3]),ARS(MD,TD)]
			print(f" pair of index {idx+1} {nxtIdx+idx+2}")
			print(f"\t\t distance symmetry: {scores[0]}")
			print(f"\t\t area symmetry: {scores[1]}")
			print(f"\t\t Aspect Ratio symmetry: {scores[2]}")
			SS = (0.7*scores[0]) + (0.2*scores[1]) + (0.1*scores[2])
			print(f" \t\t Symmetry Score: {SS}")
			if(SS >= t):
				retArr.append(CRegion(MD,TD))
	return retArr

boxes = visualizer(contours,75)
Area =[]
for rect in boxes:
    ars = (abs(rect[0][0]-rect[1][0]))/(abs(rect[0][1]-rect[1][1]))
    area = abs(rect[0][0]-rect[1][0])*abs(rect[0][1]-rect[1][1])
    
    if ((ars>4 and ars<5) and area<4500 ):
        Area.append(area)
        cv2.rectangle(roi, (rect[0][0],rect[0][1]), (rect[1][0], rect[1][1]), (0, 255, 0), 2)
# Calculating Symmetry Score and adding bounding box and text on ouput screen.
Risk = v + 0.3*(sum(Area)/100)+ 0.7*(max(Area)/100)
print(Risk)
if (Risk> 70):
    img = cv2.putText(img,f"Risk,Speed(km/h):{v}", (20,40), cv2.FONT_ITALIC , 0.5, (0, 0, 255), 2)
    cv2.imshow("original",img)
    winsound.Beep(frequency, duration)
else:
    img = cv2.putText(img,f"Safe,Speed(km/h):{v}", (20,40), cv2.FONT_ITALIC , 0.5, (0, 255, 0), 2)
    cv2.imshow("original",img)
            
#cv2.imshow("binary_opening",opening)
#cv2.imshow("binary_closing",res)
cv2.imshow("output", img)

cv2.waitKey(0)
cv2.destroyAllWindows()