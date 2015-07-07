############################################
## Import OpenCV
import numpy as np
import cv2
import serial #library function for accessing xbee module
import math
import time

ser=serial.Serial(2)

destination_position = [[0 for x in range(2)] for x in range(1)]  # destianation co-ordinates stored here
Bot_position = [[0 for x in range(2)] for x in range(2)]  # Bot markeres co-ordinates stored here

setpoint = 0            # Error should be reaches setpoint 0 using PID controller 
last_proportional = 0   # store last proportinal control value
integral = 0            # intitialise integral control value to 0

#angle = 0

Kp = 5.5                # Proportional constant
Ki = 0.1                # Integral constant
Kd = 5                  # Derivative Constant

############################################### 
'''
* Function Name:	destination
* Input:		hsv image
* Output:		destination point co-ordinate as a pixel value
* Logic:		find centroid contour of destination or master bot of pink color
*                    
* Example Call:	destiantion(hsv)
*
'''
############################################### 
def destination(hsv):
    lower = np.array([130 ,30, 160])             # Lower HSV value for destination mask
    upper = np.array([172, 210, 255])            # Higher HSV value range for destination mask
    
    mask = cv2.inRange(hsv,lower, upper)         # masking of destination colour of pink
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  # find contour of masked colour
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:1]                # sort contours decreasing order of area and store only 1 
    cv2.drawContours(frame,contours,-1,(0,0,255),7)                                     # Draw contours
    
    M = cv2.moments(contours[0])                 # calculate centroid of contour 
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    cv2.circle(frame,(cx,cy),2,(0,255,255),-1)   # centroid shows on screen

    destination_position[0][0] = cx              # Store centroids co-ordinate
    destination_position[0][1] = cy
    
###############################################
'''
* Function Name:	bot_postition
* Input:		hsv image
* Output:		slave bot's front and rear marker's coordinate as a pixel value
* Logic:		cenroid of contour of slave bot's marker of sky blue color
*                    
* Example Call:	destiantion(hsv)
*
'''                
###############################################        
def bot_position(hsv,c):

    bot_lower = np.array([73,78,57])                # Lower HSV value for bot_markers
    bot_upper = np.array([115,180,255])             # Higher HSV value range for bot_markers
    
    mask = cv2.inRange(hsv,bot_lower, bot_upper)    # masking of bot_markers of sky blue colour
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)      # find contour of masked colour
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:2]                    # sort contours decreasing order of area and store only 2 
    
    M = cv2.moments(contours[0])                    # calculate centroid of contour 
    cx1 = int(M['m10']/M['m00'])
    cy1 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx1,cy1), 2, (255,0,255), -1)
    Bot_position[0][0]=cx1                          # Store centroids co-ordinate
    Bot_position[0][1]=cy1

    M = cv2.moments(contours[1])
    cx2 = int(M['m10']/M['m00'])
    cy2 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx2,cy2), 2, (0,0,255), -1)
    Bot_position[1][0]=cx2
    Bot_position[1][1]=cy2

###############################################
'''
* Function Name:	bot_postition
* Input:		hsv image
* Output:		slave bot's front and rear marker's coordinate as a pixel value
* Logic:		cenroid of contour of slave bot's marker of sky blue color
*                    
* Example Call:	destiantion(hsv)
*
'''  
############################################### 
def dis(x1,y1,x2,y2):
        dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))     # square root function is called
        dist=int(dist)                                      # dist:stores the integer value of the distance
        return dist
    
##############################################
'''
* Function Name:	getslope
* Input:		x1,y1 co-ordinate of point one and x2,y2 co-ordinate of point two
* Output:		return slope of line formed from two points
* Logic:		use the math rule of finding slope between two points
* Example Call:	getslope(233,245,30,33)
*
'''                            
##############################################
def getslope(x1,y1,x2,y2):
        m=0
        if x2-x1!=0:                                        # checking if slope is not infinte
                m=-(float)(y2-y1)/(x2-x1)                   # using slope function of coordinate geometry
                return m
        else:
                return 50                                   # m>50 for angle>88 degrees


##############################################
'''
* Function Name:	pid_value
* Input:		error value as angle between line of bot markers an bot to destination
* Output:		correction in the value which added to motor speed
* Logic:		proportional, integral and derivative are give commulative correction
* Example Call:	pid_value(-30)
*
'''  
##############################################
def pid_value(error):
        global integral,last_proportional                   # globalize the variables 
        
        proportional = error - setpoint                     # proportional control value
        
        integral = integral + proportional                  # integral control value
        if(integral < -255):                                # clamped integral control value
            integral = -255
        if(integral > 255):
            integral = 255
            
        derivative = proportional - last_proportional       # derivative control value

        last_proportional = proportional                    # store last proportional value
        
        correction = Kp*proportional + Ki*integral + Kd*derivative      # correction value 

        return correction
        
##############################################
'''
* Function Name:	orientmove
* Input:		slope bot to destination, slope of botmarkers,distance_centre2cell,distance_other2cell,bot_distance 
* Output:		xbee commands to control the firebird V
* Logic:		pid_correction value is added to left and right motor speeds according to turn decisions
*
''' 
##############################################
def orientmove(slope_bot2cell,slope_botmarkers,distance_centre2cell,distance_other2cell,bot_distance):

        if distance_other2cell < 50 :                                       # check slave bot reaches destination if yes stop
                ser.write("\xfd")
                print "Reached Near To Master Bot"                      
                                
        else:
                
                if slope_bot2cell*slope_botmarkers!=-1 :                    # check angle between slopes is not 90
                        theta=math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))          # using tan inverse from math to finding angle between two lines
                        angle = int(math.degrees(theta))                                                                # convert angle from radian to degrees

                         if pow(distance_other2cell,2) > pow(distance_centre2cell,2) + pow(bot_distance,2):             # if other marker is farther from the grid cell's centre then error value is changed
                                 if angle < 0:
                                     angle = 180 + angle                                                    
                                 else:
                                     angle = -180 + angle
                                     
                        pid_correction = int(pid_value(angle))              # error value send to pid_controller to process
                        
                        if(pid_correction < -255):                          # clamped pid output value
                            pid_correction = -255
                        if(pid_correction >255):
                            pid_correction = 255

                        if(pid_correction > -5 and pid_correction < 5):     # go forward if in range of setpoint
                           ser.write("\xf8") 
                        elif(pid_correction < -5):                          # take right if error is negative
                           if pid_correction > -250:
                               pid_correction = 255 + pid_correction
                               pid_correction = pid_correction/2+pid_correction%2
                               ser.write(chr(pid_correction))               # add correction to motor speed
                               ser.write("\xff")                            # command to turn right
                               
                           else :                                           # fast right if error exceed min range
                               ser.write(chr(0))
                               ser.write("\xfb")  # right
                               
                        else:
                           if pid_correction < 250 :                        # take left if error is positive
                               pid_correction = 255-pid_correction
                               pid_correction = pid_correction/2+pid_correction%2
                               ser.write(chr(pid_correction))               # add correction to motor speed
                               ser.write("\xfe")                            # command to turn left
                               
                           else :                                           # fast left if error exceed max range
                               ser.write(chr(0))                            
                               ser.write("\xfc")                            
                                           
                else:
                    return 0                # return 0 if angle is 90

##############################################
'''
* Function Name:	main function
* Logic:		capture video from cam and convert it to hsv for further video processing
*
'''                  
##############################################
if __name__ == "__main__":

    cap=cv2.VideoCapture(1)                 # Capture video
    while(1):
        time.sleep(1/80)                    # time between frames sampling is 1/80 sec.
        ret,frame=cap.read()                # read frames from video
        
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)         # convert rgb coloured image to rgb image
        
        bot_position(hsv)                                 # call bot_position function
        destination(hsv)                                  # call destination function

        cv2.line(frame,(destination_position[0][0],destination_position[0][1]),(Bot_position[0][0],Bot_position[0][1]),(255,255,0), 1)      # draw line between bot rear to destination
        cv2.line(frame,(Bot_position[0][0],Bot_position[0][1]),(Bot_position[1][0],Bot_position[1][1]),(0,255,255), 1)                      # draw line between bot markers

        destination_slope = getslope(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])           # find the slope of line passing through bot rear and destination points 
        bot_slope = getslope(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])                                   # find the slope of line passing through bot rear and bot front markers

        distance_centre2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])      # find distance bot rear to destination
        distance_other2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[1][0],Bot_position[1][1])       # find distance bot front to destination
        bot_distance = dis(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])                                     # find distance between bot markers

        if(orientmove(destination_slope,bot_slope,distance_centre2destination,distance_other2destination,bot_distance)==1):                 # send values to find the orientation and movement of slave bot
           break 
        
        cv2.imshow('frame',frame)           # show output on frame
        if cv2.waitKey(100)==27:            # exit from the loop or video
            break

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
#################################################
