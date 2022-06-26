#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np;
from std_msgs.msg import String
flag = ""
#callback function for camera subscriber, called by the camera subscriber for every frame.
def callback(data):

 bridge = CvBridge()

 #Convert incoming image from a ROS image message to a CV image that open CV can process.
 cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
 #Display the converted cv image, this is the raw camera feed data.
 cv2.imshow("Raw Camera Feed", cv_image)
 
 #find red block
 hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
 LowerR = (0, 81, 102)
 UpperR = (12, 255, 255)
 maskR = cv2.inRange(hsv, LowerR, UpperR)
 maskR = cv2.erode(maskR, None, iterations=2)
 maskR = cv2.dilate(maskR, None, iterations=2)
 cntsR = cv2.findContours(maskR.copy(), cv2.RETR_EXTERNAL,
 cv2.CHAIN_APPROX_SIMPLE)
 cntsR = cntsR[0]
 for c in cntsR:
 # compute the center of the contour
  M = cv2.moments(c)
  rX = int(M["m10"] / M["m00"])
  rY = int(M["m01"] / M["m00"])
 # draw the contour and center of the shape on the image
  
  cv2.circle(cv_image, (rX, rY), 2, (255, 255, 255), -1)

 
 r = [rX, rY]
 
 #find yellow block
 hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
 LowerY = (20, 102, 160)
 UpperY = (30, 255, 255)
 maskY = cv2.inRange(hsv, LowerY, UpperY)
 maskY = cv2.erode(maskY, None, iterations=1)
 maskY = cv2.dilate(maskY, None, iterations=1)
 cntsY = cv2.findContours(maskY.copy(), cv2.RETR_EXTERNAL,
 cv2.CHAIN_APPROX_SIMPLE)
 cntsY = cntsY[0]
 for c in cntsY:
 # compute the center of the contour
  M = cv2.moments(c)
  if M["m00"] != 0:

   yx = int(M["m10"] / M["m00"])
   yy = int(M["m01"] / M["m00"])
  else:
   yx, yy = 0, 0
 # draw the contour and center of the shape on the image
  
  cv2.circle(cv_image, (yx, yy), 2, (255, 255, 255), -1)
 
 
 o = [yx, yy]
 
 #find green block
 hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
 LowerG = (60, 102, 160)
 UpperG = (70, 255, 255)
 maskG = cv2.inRange(hsv, LowerG, UpperG)
 maskG = cv2.erode(maskG, None, iterations=1)
 maskG = cv2.dilate(maskG, None, iterations=1)
 cntsG = cv2.findContours(maskG.copy(), cv2.RETR_EXTERNAL,
 cv2.CHAIN_APPROX_SIMPLE)
 cntsG = cntsG[0]
 for c in cntsG:
 # compute the center of the contour
  M = cv2.moments(c)
  gX = int(M["m10"] / M["m00"])
  gY = int(M["m01"] / M["m00"])
 # draw the contour and center of the shape on the image
  
  cv2.circle(cv_image, (gX, gY), 2, (255, 255, 255), -1)
 
 
 g = [gX, gY]
 
 #find blue block
 hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
 LowerB = (120, 102, 160)
 UpperB = (125, 255, 255)
 maskB = cv2.inRange(hsv, LowerB, UpperB)
 maskB = cv2.erode(maskB, None, iterations=1)
 maskB = cv2.dilate(maskB, None, iterations=1)
 cntsB = cv2.findContours(maskB.copy(), cv2.RETR_EXTERNAL,
 cv2.CHAIN_APPROX_SIMPLE)
 cntsB = cntsB[0]
 for c in cntsB:
 # compute the center of the contour
  M = cv2.moments(c)
  bX = int(M["m10"] / M["m00"])
  bY = int(M["m01"] / M["m00"])
 # draw the contour and center of the shape on the image
  
  cv2.circle(cv_image, (bX, bY), 2, (255, 255, 255), -1)
 
 b = [bX, bY]
 
 cv2.imshow("MaskR", maskR)
 cv2.imshow("MaskB", maskB) 
 cv2.imshow("Mask G", maskG)
 cv2.imshow("Mask Y", maskY) 
 cv2.imshow("Img", cv_image)
 

 global flag
 if flag == "y":
  positions = calc(r,g,b,o)
  moveDisks(positions, 0, 2)
  flag = ""
 
 
 
 #3ms wait
 cv2.waitKey(3)

def newcallback(data):
 rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
 global flag
 flag = data.data
def listener():
 rospy.Subscriber('chatter2', String, newcallback)
 
def moveDisks(diskPositions, largestToMove, targetPeg):
 for badDisk in range(largestToMove, len(diskPositions)):

  currentPeg = diskPositions[badDisk]         
  if currentPeg != targetPeg:
            #found the largest disk on the wrong peg

            #sum of the peg numbers is 3, so to find the other one...
   otherPeg = 3 - targetPeg - currentPeg

            #before we can move badDisk, we have get the smaller ones out of the way
   moveDisks(diskPositions, badDisk+1, otherPeg)
   
   print ("Move {0} from {1} to {2}".format(badDisk, currentPeg, targetPeg))
   diskPositions[badDisk]=targetPeg

            #now we can put the smaller ones in the right place
   moveDisks(diskPositions, badDisk+1, targetPeg)
    
    
   break;
   

  


def calc(r,g,b,o):
 positions = [0,0,0,0]
 if(r[0] >= 53) and (r[0] < 110):
  positions[0] = 0
 elif(r[0] >= 140) and (r[0] < 200):
  positions[0] = 1
 elif(r[0] >= 230):
  positions[0] = 2

 if(g[0] >= 53) and (g[0] < 110):
  positions[1] = 0
 elif(g[0] >= 140) and (g[0] < 200):
  positions[1] = 1
 elif(g[0] >= 230):
  positions[1] = 2

 if(b[0] >= 53) and (b[0] < 110):
  positions[2] = 0
 elif(b[0] >= 140) and (b[0] < 200):
  positions[2] = 1
 elif(b[0] >= 230):
  positions[2] = 2

 if(o[0] >= 53) and (o[0] < 110):
  positions[3] = 0
 elif(o[0] >= 140) and (o[0] < 200):
  positions[3] = 1
 elif(o[0] >= 230):
  positions[3] = 2
 
 return positions
 

if __name__ == '__main__':
 rospy.init_node('Colour_Detection', anonymous=False)
 
 
 
#create subscriber to the right hand camera, each frame recieved calls the callback function

camera_sub = rospy.Subscriber("/bob/camera/outer",Image,callback)
listener() 

#prevents program from exiting, allowing subscribers and publishers to keep operating
#in our case that is the camera subscriber and the image processing callback function
rospy.spin()
