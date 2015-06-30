############################################
## Import OpenCV
import numpy as np
import cv2
import heapq ##priority queue
import serial #library function for accessing xbee module
import math



ser=serial.Serial(2)

Kp = 2
Ki = 0
Kd = 0
#########################################
def send_value(value):
    data1 = value/100
    data2 = value%100/10
    data3 = value%10
    if data1 == 0:
        ser.write("0")
    elif data1 == 1:
        ser.write("1")
    elif data1 == 2:
        ser.write("2")
    elif data1 == 3:
        ser.write("3")
    elif data1 == 4:
        ser.write("4")
    elif data1 == 5:
        ser.write("5")
    elif data1 == 6:
        ser.write("6")
    elif data1 == 7:
        ser.write("7")
    elif data1 == 8:
        ser.write("8")
    #elif data1 == 9:
    else : 
        ser.write("9")    

    if data2 == 0:
        ser.write("0")
    elif data2 == 1:
        ser.write("1")
    elif data2 == 2:
        ser.write("2")
    elif data2 == 3:
        ser.write("3")
    elif data2 == 4:
        ser.write("4")
    elif data2 == 5:
        ser.write("5")
    elif data2 == 6:
        ser.write("6")
    elif data2 == 7:
        ser.write("7")
    elif data2 == 8:
        ser.write("8")
    #elif data2 == 9:
    else :
        ser.write("9")

    if data3 == 0:
        ser.write("0")
    elif data3 == 1:
        ser.write("1")
    elif data3 == 2:
        ser.write("2")
    elif data3 == 3:
        ser.write("3")
    elif data3 == 4:
        ser.write("4")
    elif data3 == 5:
        ser.write("5")
    elif data3 == 6:
        ser.write("6")
    elif data3 == 7:
        ser.write("7")
    elif data3 == 8:
        ser.write("8")
    #elif data3 == 9:
    else :
        ser.write("9")

############################
def dis(x1,y1,x2,y2):
        dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))#square root function is called
        dist=int(dist)#dist:stores the integer value of the distance
        #print dist
        return dist
    
##########################
# converting grid coordinates into pixels
'''
* Function Name:	grid_to_pixel
* Input:		grid_x: integer which stores grid cell x coordinates
*                           grid_y: integer which stores grid cell y coordinates
*                           height: integer, stores height of grid cell
*                           width: integer, stores width of grid cell
* Output:		returns grid cell's pixel coordinates
* Logic:		converts grid cell's centre coordinates into pixel coodinates
*                       
* Example Call:	gridtopixel(3,3,30,33)
*
'''
def grid_to_pixel(grid_x,grid_y,height,width):
        #accessing centre of grid cell by multiplying grid coordinates with respective height and width and adding their halves
        pixel_x=grid_x*height+height/2 #pixel_x: stores the integer value of pixel's x coodinate
        pixel_y=grid_y*width+width/2 #pixel_y: stores the integer value of pixel's y coodinate
        return pixel_x,pixel_y
########################

##############################################
'''
* Function Name:	getcoor
* Input:		pixel_x: integer which stores pixel's x coordinates
*                           pixel_y: integer which stores pixel's y coordinates
*                           heigtht: integer, stores height of grid cell
*                           width: integer, stores width of grid cell
* Output:		returns grid cell's coordinates
* Logic:		Converts pixel coordinates into their specific grid coordinates under which those pixels lies.
*                       
* Example Call:	getcoor(233,245,30,33)
*
'''   
################################################
def get_coordinate(x,y):
        '''
        cx=x/n#(int)(round(x/m))
        cy=y/n#(int)(round(y/n))
        return cx,cy
        '''
        #img=cv2.imread(filename) ##getting input image
        X=0
        Y=0
        for i in range(0, grid_line_x): ##drawing lines
                X=X+line_widthn
                Y=0
                for j in range(0, grid_line_y): ##drawing lines
                        Y=Y+line_widthm
                        #print X,Y
                        if x<=X and y<=Y:
                                return i,j
                                break
                            
#############################
def getslope(x1,y1,x2,y2):
        m=0
        if x2-x1!=0:#checking if slope is not infinte
                m=-(float)(y2-y1)/(x2-x1)#using slope function of coordinate geometry
                return m
        else:
                return 50#m>50 for angle>88 degrees or (180-88), in case slope is approaching infinite


###############################
def pid_value(error):
        proportional = error - setpoint
        integral = integral + proportional
        derivative = last_proprtional - proportional

        last_proportional = proportional

        correction = Kp*proportional + Ki*integral + Kd*derivative
        return correction
        
        
############################    
def orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x,bot_grid_y,route_x,route_y,distance_centre2cell,distance_other2cell):
        print bot_grid_x,bot_grid_y,route_x,route_y
        print stepper
        bot_grid_x, bot_grid_y = grid_to_pixel(bot_grid_x,bot_grid_y,line_widthm,line_widthn)
        route_x,route_y = grid_to_pixel(route_x,route_y,line_widthm,line_widthn)
        
        if bot_grid_x==route_x and bot_grid_y==route_y: #check if bot has reached next coordinate
                #ser.write("5")
                print "Hello"
                ser.write("7")
                return 1
        else:
                
                if slope_bot2cell*slope_botmarkers!=-1 :
                        theta=math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))
                        angle = math.degrees(theta) # print theta in degree
                        pid_correction = pid_value(angle)

                                                
                        if distance_other2cell>distance_centre2cell: #if other marker is farther from the grid cell's centre then move it closer with fast turns
                                 if theta<20: #20 for theta greater than 90 degrees, here theta is in radians
                                       ser.write("A")  #fast right turn
                                 else:
                                       ser.write("D")   #fast left turn
                        else:
                                 if(pid_correction == 0):
                                       ser.write("F")
                                 elif(pid_correction < 0):
                                       send_value((-1)*pid_correction)
                                       ser.write("R")
                                 else:
                                       send_value(pid_correction)
                                       ser.write("L")
                                
                                #ser.write(com) #send command
                        
                else:
                        return 0
