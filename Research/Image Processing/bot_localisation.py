
############################################
## Import OpenCV
import numpy as np
import cv2
import serial #library function for accessing xbee module
import math

ser=serial.Serial(2)
destination_position = [[0 for x in range(2)] for x in range(2)]
Bot_position = [[0 for x in range(2)] for x in range(2)]

def destination(hsv):
    lower = np.array([140 ,35, 255]) 
    upper = np.array([160, 255, 255])

    mask = cv2.inRange(hsv,lower, upper)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:4] ##obstacles
    #cv2.drawContours(frame,contours,-1,(0,0,255),7)
    #cv2.imshow('yellow',frame)
    
    M = cv2.moments(contours[0])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    #print "Centroid = ", cx, ", ", cy
    cv2.circle(frame,(cx,cy), 5, (0,255,255), -1)

    destination_position[0][0] = cx
    destination_position[0][1] = cy

    print cx,cy, "destination"

############################
def dis(x1,y1,x2,y2):
        dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))#square root function is called
        dist=int(dist)#dist:stores the integer value of the distance
        #print dist
        return dist

#############################
def getslope(x1,y1,x2,y2):
        m=0
        if x2-x1!=0:#checking if slope is not infinte
                m=-(float)(y2-y1)/(x2-x1)#using slope function of coordinate geometry
                return m
        else:
                return 50#m>50 for angle>88 degrees or (180-88), in case slope is approaching infinite

############################    
def orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x,bot_grid_y,route_x,route_y,distance_centre2cell,distance_other2cell):
        if bot_grid_x==route_x and bot_grid_y==route_y: #check if bot has reached next coordinate
                ser.write("5")
                #print "Hello"
                #ser.write("7")
                return 1
        else:
                
                if slope_bot2cell*slope_botmarkers!=-1 :
                        theta=math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))
                        #print theta
                        if distance_other2cell>distance_centre2cell: #if other marker is farther from the grid cell's centre then move it closer with fast turns
                                 if theta<20: #20 for theta greater than 90 degrees, here theta is in radians
                                       ser.write("A")  #fast right turn
                                 else:
                                       ser.write("D")   #fast left turn
                        elif (theta<-0.18 or theta>0.18): #align the bot # if marker is closer to grid cell's centre, then try to align it with the cell's centre
                                #com=1
                                if theta<-0.18:
                                       ser.write("6")  #right turn
                                else:
                                       ser.write("4")   #left turn
                                #com = raw_input()
                                
                                #ser.write(com) #send command
                        
                        
                        else:#if theta approaches zero, it means bot is nearly alighned with the next grid cell and it can move forward
                               ser.write("8") #forward bot
                return 0
            
##########################################         
def bot_position(hsv):
    
    bot_lower = np.array([0,45,255])
    bot_upper = np.array([40,255,255])
    
    #####front end masking and centroid
    mask = cv2.inRange(hsv,bot_lower, bot_upper)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:2]
    #contours,length=areacon(contours,700,300)
    #contours=sorted(contours, key = cv2.contourArea, reverse = True)[:length]
    #cv2.drawContours(frame,contours,-1,(100,100,255),1)
    cv2.imshow('bot',mask)
    #print "len ",len(contours)
    M = cv2.moments(contours[0])
    cx1 = int(M['m10']/M['m00'])
    cy1 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx1,cy1), 5, (255,0,255), -1)
    Bot_position[0][0]=cx1
    Bot_position[0][1]=cy1
    #print cx1,cy1
    #print Bot_position[0][0],
    M = cv2.moments(contours[1])
    cx2 = int(M['m10']/M['m00'])
    cy2 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx2,cy2), 5, (0,0,255), -1)
    Bot_position[1][0]=cx2
    Bot_position[1][1]=cy2

    print cx1,cy1, "1"
    print cx2,cy2, "2"

if __name__ == "__main__":    
    cap=cv2.VideoCapture(1)
    while(1):
        ## Read the image
        #img = cv2.imread('output_image.jpg')
        ret,frame=cap.read()
        h,k,l=frame.shape
        print h,k,l,"frame size"
        print frame[370][220]
        print frame[350][260]
        print frame[310][240]
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        bot_position(hsv)
        destination(hsv)
        cv2.line(frame,(destination_position[0][0],destination_position[0][1]),(Bot_position[0][0],Bot_position[0][1]),(255,255,0), 6)
        cv2.line(frame,(Bot_position[0][0],Bot_position[0][1]),(Bot_position[1][0],Bot_position[1][1]),(0,255,255), 6)
        destination_slope = getslope(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        bot_slope = getslope(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])
        distance_centre2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        distance_other2destination = dis(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])

        
        print bot_slope,"bot",destination_slope,"destination"
        orientmove(destination_slope,bot_slope,Bot_position[0][0],Bot_position[0][1],destination_position[0][0],destination_position[0][1],distance_centre2destination,distance_other2destination)
        cv2.imshow('frame',frame)
        if cv2.waitKey(100)==27:
            break

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
#################################################
