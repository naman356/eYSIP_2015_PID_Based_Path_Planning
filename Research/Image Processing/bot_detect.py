
############################################
## Import OpenCV
import numpy as np
import cv2
import heapq ##priority queue
import serial #library function for accessing xbee module
import math
import time
import thread
#from path import *
#from movements import *

ser=serial.Serial(2)

grid_line_x = 21
grid_line_y = 21
path_sample = 0
stepper = 0
grid_map = [ [ 0 for i in range(grid_line_y) ] for j in range(grid_line_x) ]

destination_position = [[0 for x in range(2)] for x in range(2)]
Bot_position = [[0 for x in range(2)] for x in range(2)]

grid_map = [ [ 0 for i in range(grid_line_y) ] for j in range(grid_line_x) ]

flag = 1
last_route_length = 0
setpoint = 0
last_proportional = 0
integral = 0
angle = 0
c=0
Kp = 5
Ki = 0.05
Kd = 5   

def destination(hsv,c):
    if(c==1):
        lower = np.array([130,40,248]) # for image
        upper = np.array([179,255,255])

     #   lower = np.array([130,50,248]) # for image
     #   upper = np.array([179,255,255])

    else:    
        lower = np.array([80 ,60, 250]) 
        upper = np.array([179, 255, 255])
    
    mask = cv2.inRange(hsv,lower, upper)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:1] ##obstacles
    cv2.drawContours(frame,contours,-1,(0,0,255),7)
    
    M = cv2.moments(contours[0])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    cv2.circle(frame,(cx,cy), 2, (0,255,255), -1)

    destination_position[0][0] = cx
    destination_position[0][1] = cy
                
##########################################         
def bot_position(hsv,c):

    if(c==1):
        # bot_lower = np.array([20,25,230]) # for image 
        # bot_upper = np.array([40,255,255])

        bot_lower = np.array([20,25,230]) # for image 
        bot_upper = np.array([40,255,255])
    else:
        bot_lower = np.array([0,70,240])
        bot_upper = np.array([45,255,255])

    #####front end masking and centroid
    mask = cv2.inRange(hsv,bot_lower, bot_upper)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:2]
    #contours,length=areacon(contours,700,300)
    #contours=sorted(contours, key = cv2.contourArea, reverse = True)[:length]
    #cv2.drawContours(frame,contours,-1,(100,100,255),1)

    M = cv2.moments(contours[0])
    cx1 = int(M['m10']/M['m00'])
    cy1 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx1,cy1), 2, (255,0,255), -1)
    Bot_position[0][0]=cx1
    Bot_position[0][1]=cy1

    M = cv2.moments(contours[1])
    cx2 = int(M['m10']/M['m00'])
    cy2 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx2,cy2), 2, (0,0,255), -1)
    Bot_position[1][0]=cx2
    Bot_position[1][1]=cy2

############################################
def wall_detection(hsv,c):
    if c==1 :
        lower = np.array([45 ,30, 170],np.uint8)   # for image
        upper = np.array([110, 170, 255],np.uint8)

        # lower = np.array([45 ,30, 170],np.uint8)   # for image
        # upper = np.array([110, 170, 255],np.uint8)

    else :
        lower = np.array([45 ,75, 30],np.uint8)
        upper = np.array([90, 255, 255],np.uint8)
 
    mask = cv2.inRange(hsv,lower, upper)

    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:8]

    cv2.fillPoly(mask,contours, (255,255,255))

    kernel = np.ones((40,40),np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    erosion = cv2.erode(mask,kernel,iterations = 1)
    dilation = cv2.dilate(closing,kernel,iterations = 1)#obstacle = cv2.dilate(closing,kernel,iterations = 1)

    return dilation

##############################################
    
def grid_draw(frame,m,n): ##filename is image filename with full file path, n is grid of n lines
    for x in range(0, m): ##drawing lines
        X=x*line_widthm
        cv2.line(frame,(0,X),(k,X),(0,0,0), 2)#lines is red color, bgr format
    for y in range(0, n): ##drawing lines
        Y=y*line_widthn
        cv2.line(frame,(Y,0),(Y,h),(0,0,0), 2)
    return (frame)

############################################
def solve(start,finish,frame): #no heuristics used
     """Find the shortest path from START to FINISH."""
     heap=[]
     link = {} # parent node link
     g = {} # shortest path to a current node
     g[start] = 0 #initial distance to node start is 0
    
     link[start] = None #parent of start node is none
    
    
     heapq.heappush(heap, (0, start))
    
     while True:
         f, current = heapq.heappop(heap) ##taking current node from heap

         if current == finish:
             name='Shortest Path, image#'
             i=int(100*np.random.rand())
             name=name+str(i)
             route=build_path(start, finish, link)
             '''####Drawing path , just for pictorial representation######
             for i in range(1,len(route)):
                cv2.line(frame,(route[i-1].y*line_widthn+line_widthn/2,route[i-1].x*line_widthm+line_widthm/2),(route[i].y*line_widthn+line_widthn/2,route[i].x*line_widthm+line_widthm/2),(200,200,0), 3)
             cv2.imshow('path',frame)'''
             ############################
             return g[current], route[1:len(route)]
            
        
         moves = current.get_moves()
         cost = g[current]
         for mv in moves:
             if grid_map[mv.x][mv.y]==1: #bypass obstacles
                 continue
                 #mv is the neighbour of current cell, in all maximum 4 neighbours will be there
             if  (mv not in g or g[mv] > cost + 1): #check if mv is already visited or if its cost is higher than available cost then update it
                 g[mv] = cost + 1
                
                 link[mv] = current #storing current node as parent to mv 
                 heapq.heappush(heap, (g[mv], mv)) ##adding updated cost and visited node to heap

######################################################    
def build_path(start, finish, parent):
    
    #create path from start to finish

    x = finish ##back tracking the path from goal to start
    xs = [x]
    while x != start: #going back
        x = parent[x]
        xs.append(x)
    xs.reverse()
 
    return xs

###########################################################
class GridPoint(object):
    """Represent a position on a grid."""
    def __init__(self, x, y): #self referencing x and  y coordinates
        self.x = x
        self.y = y

    def __hash__(self): #returning hash value of the GridPoint object
        return hash((self.x, self.y))

    def __repr__(self):                         #returns values stored in current object, values are x and y coordinates
        return "(%d,%d)" % (self.y+1, self.x+1)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def get_moves(self): ##taking current node coordinates to find neighbours of it
        
        
        if self.x>=0 and self.x<=len(grid_map)-1 and self.y>=0 and self.y<=len(grid_map)-1:
            if self.x + 1<len(grid_map):
                yield GridPoint(self.x + 1, self.y)
            if self.y + 1<len(grid_map):  
                yield GridPoint(self.x, self.y + 1)
            if self.x - 1>=-1:
                yield GridPoint(self.x - 1, self.y)
            if self.y - 1>=-1:
                yield GridPoint(self.x, self.y - 1)
                

            #################################################
            '''if self.x + 1<len(grid_map) and self.y + 1<len(grid_map):
                yield GridPoint(self.x+1, self.y+1)
            if self.y + 1<len(grid_map) and  self.x - 1>-1:  
                yield GridPoint(self.x-1, self.y + 1)    
            if self.x - 1>-1 and self.y - 1>-1:
                yield GridPoint(self.x-1, self.y-1)
            if self.y - 1>-1 and self.x + 1<len(grid_map):
                yield GridPoint(self.x+1, self.y - 1)'''

#############################################################
def grid_map_of_walls(walls,grid_line_x,grid_line_y):
    '''
    frame-- a single test image as input argument
    route_length  -- returns the single integer specifying the route length
    '''
    global grid_map
    global last_grid_map

    #creating 10x10 matrix space map with black as obstable and other colors as paths.
    for x in range(0,grid_line_x-1):
        X=x*line_widthm+(line_widthm/2)
        for y in range(0,grid_line_y-1):
            Y=y*line_widthn+(line_widthn/2)
            if walls[X,Y]>=250 or x==0 or x==grid_line_x-2 or y==0 or y==grid_line_y-2: # and frame[X,Y,1]>=70 and frame[X,Y,2]>=70: #obstacle black ,bgr value(0,0,0)
                grid_map[x][y]=1
                cv2.circle(frame,(Y,X), 1, (0,50,200), -1)
            continue
    last_grid_map = grid_map


####################################
def route_path_draw(frame):
    ####Drawing path , just for pictorial representation######
    for i in range(1,len(route_path)):
        cv2.line(frame,(route_path[i-1].y*line_widthn+line_widthn/2,route_path[i-1].x*line_widthm+line_widthm/2),(route_path[i].y*line_widthn+line_widthn/2,route_path[i].x*line_widthm+line_widthm/2),(200,200,0), 1)
        #cv2.imshow('path',frame)
           
############################
def dis(x1,y1,x2,y2):
        dist=math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))#square root function is called
        dist=int(dist)#dist:stores the integer value of the distance
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
        pixel_x=grid_x*height+height/2 # stores the integer value of pixel's x coodinate
        pixel_y=grid_y*width+width/2 # stores the integer value of pixel's y coodinate
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
        
        X=0
        Y=0
        for i in range(0, grid_line_x): ##drawing lines
                X=X+line_widthn
                Y=0
                for j in range(0, grid_line_y): ##drawing lines
                        Y=Y+line_widthm
                        if x<=X and y<=Y:
                                return i,j
                                break
                            
#############################
def getslope(x1,y1,x2,y2):
        m=0
        if x2-x1!=0: # checking if slope is not infinte
                m=-(float)(y2-y1)/(x2-x1) # using slope function of coordinate geometry
                return m
        else:
                return 50 # m>50 for angle>88 degrees or (180-88), in case slope is approaching infinite


###############################
def pid_value(error):
        global integral,last_proportional
        proportional = error - setpoint
        
        integral = integral + proportional
        if(integral < -255):
            integral = -255
        if(integral > 255):
            integral = 255
        derivative = proportional - last_proportional

        last_proportional = proportional
        
        #print proportional,"proportional",integral,"integral",derivative,"derivative"
        correction = Kp*proportional + Ki*integral + Kd*derivative

        return correction
    
##################################
def modified_angle(angle):
        angle = angle/10
        angle = angle*10
        return angle
    
############################    
def orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x,bot_grid_y,route_x,route_y,distance_centre2cell,distance_other2cell,bot_distance):

        global c
        global start,stop
        
        # bot_grid_x, bot_grid_y = grid_to_pixel(bot_grid_x,bot_grid_y,line_widthm,line_widthn)
        # route_x,route_y = grid_to_pixel(route_x,route_y,line_widthm,line_widthn)
        
        if bot_grid_x==route_x and bot_grid_y==route_y: #check if bot has reached next coordinate
        #if distance_other2cell < 33 :
                #ser.write("\xfd")
                #print "reached"
                return 1
        else:
                
                if slope_bot2cell*slope_botmarkers!=-1 :
                        theta=math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))
                        angle = int(math.degrees(theta)) # print theta in degree

                        if pow(distance_other2cell,2) > pow(distance_centre2cell,2) + pow(bot_distance,2): #if other marker is farther from the grid cell's centre then move it closer with fast turns
                                 if angle < 0 :
                                     angle = 180 + angle
                                 else :
                                     angle = -180 + angle
                        pid_correction = int(pid_value(angle))
                        
                        if(pid_correction < -255):
                            pid_correction = -255
                        if(pid_correction >255):
                            pid_correction = 255

                        '''if distance_other2cell == 0: #if other marker is farther from the grid cell's centre then move it closer with fast turns
                                 if(pid_correction > 0 and pid_correction < 255):
                                       #ser.write(chr(0))
                                       ser.write("\xfb") # right
                                       print "Opp right",pid_correction
                                 if(pid_correction < 0 and pid_correction > -255):
                                       #ser.write(chr(0))
                                       ser.write("\xfc") # left
                                       print "Opp left",pid_correction'''
                                 
                        if 1 == 1:
                                 if(pid_correction > -5 and pid_correction < 5):
                                       ser.write("\xf8")
                                 elif(pid_correction < -5):
                                       if pid_correction > -250:
                                           pid_correction = 255 + pid_correction
                                           pid_correction = pid_correction/2+pid_correction%2
                                           ser.write(chr(pid_correction))
                                           ser.write("\xff")  # slow right
                                           
                                       else :
                                           ser.write(chr(0))
                                           ser.write("\xfb")  # right
                                           
                                 else:
                                       if pid_correction < 250 :
                                           pid_correction = 255-pid_correction
                                           pid_correction = pid_correction/2+pid_correction%2
                                           ser.write(chr(pid_correction))
                                           ser.write("\xfe")  # slow left
                                           
                                       else :
                                           ser.write(chr(0))
                                           ser.write("\xfc")  # left
                                           
                else:
                        return 0

##############################################
def bot_traverse(route_path,destination,frame):
    global stepper
    
    if stepper<len(route_path)-1:
        X,Y=grid_to_pixel(route_path[stepper].x,route_path[stepper].y,line_widthm,line_widthn)# X,Y are pixels of next grid coor

        rear_bot_x,rear_bot_y=get_coordinate(Bot_position[0][0],Bot_position[0][1])
        front_bot_x,front_bot_y=get_coordinate(Bot_position[1][0],Bot_position[1][1])

        d1 = dis(Bot_position[0][0],Bot_position[0][1],Y,X)
        d2 = dis(Bot_position[1][0],Bot_position[1][1],Y,X)
        d3 = dis(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])

        #d1 = dis(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        #d2 = dis(destination_position[0][0],destination_position[0][1],Bot_position[1][0],Bot_position[1][1])

        mid_x=(Bot_position[0][0]+Bot_position[1][0])/2#mid point of bot center and other point
        mid_y=(Bot_position[0][1]+Bot_position[1][1])/2#mid point of bot center and other point

        bot_grid_x,bot_grid_y=get_coordinate(mid_x,mid_y)
        
        cv2.circle(frame,(Y,X), 5, (255,100,100), -1)

        m1= getslope(Bot_position[0][0],Bot_position[0][1],Y,X)
        m2= getslope(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])

        if orientmove(m1,m2,rear_bot_x+1,rear_bot_y+1,route_path[stepper].y+1,route_path[stepper].x+1,d1,d2,d3)==1: #bot reaches next coor
            stepper=stepper+1
            
    else:
        ser.write("\xfd")
        print "reached"
        
############################################################
def bot_route(frame):

    global path_sample
    global route_length, route_path
    global grid_start,grid_end
    global last_route_length
    global last_route_path
    
    grid_start = GridPoint((Bot_position[0][1]/line_widthm),(Bot_position[0][0]/line_widthn)) ##reversing coordinates so that it can be compatible with coordinate system of matrix
    grid_end = GridPoint((destination_position[0][1]/line_widthm),(destination_position[0][0]/line_widthn))

    route_length, route_path = solve(grid_start,grid_end,frame)
       
    if last_route_length == route_length :
        route_path = last_route_path

    print route_path
    path_sample = path_sample - 1
    last_route_length = route_length
    last_route_path = route_path

################################################################
def route_sample(frame):
    while(1):
        bot_route(frame)
        time.sleep(5)
        
####################################################
if __name__ == "__main__":
    global h,k,l
    global line_widthm,line_widthn
    
    cap=cv2.VideoCapture(1)
    ret,frame=cap.read()
    k=1000
    while k :
        k= k-1

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    cv2.imwrite('frame3.jpeg',frame)
    
    h,k,l=frame.shape
    line_widthm=h/(grid_line_x-1)
    line_widthn=k/(grid_line_y-1)

    destination(hsv,1)
    bot_position(hsv,0)
                          
    walls=wall_detection(hsv,0)
    grid_map_of_walls(walls,grid_line_x,grid_line_y)

    destination_x,destination_y = get_coordinate(destination_position[0][0],destination_position[0][1])
        
    #thread.start_new_thread(route_sample,(hsv, ))
    
    #route_sample()
    bot_route(frame)
    k=0
    while(1):
        ret,frame=cap.read()

        time.sleep(1/80)
        ret,frame=cap.read()
        
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        bot_position(hsv,0)
        destination(hsv,0)

        h,k,l=frame.shape
        line_widthm=h/(grid_line_x-1)
        line_widthn=k/(grid_line_y-1)

        walls=wall_detection(hsv,0)
        grid_map_of_walls(walls,grid_line_x,grid_line_y)

        cv2.line(frame,(destination_position[0][0],destination_position[0][1]),(Bot_position[0][0],Bot_position[0][1]),(255,255,0), 1)
        cv2.line(frame,(Bot_position[0][0],Bot_position[0][1]),(Bot_position[1][0],Bot_position[1][1]),(0,255,255), 1)

        destination_slope = getslope(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        bot_slope = getslope(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])

        distance_centre2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        distance_other2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[1][0],Bot_position[1][1])

        destination_x,destination_y = get_coordinate(destination_position[0][0],destination_position[0][1])
        rear_bot_x,rear_bot_y=get_coordinate(Bot_position[0][0],Bot_position[0][1])
        front_bot_x,front_bot_y=get_coordinate(Bot_position[1][0],Bot_position[1][1])

      
        #bot_route(frame)

        route_path_draw(frame)

        #grid_draw (frame,grid_line_x,grid_line_y)

        bot_traverse(route_path,grid_end,frame)
        
        '''if(orientmove(destination_slope,bot_slope,front_bot_x,front_bot_y,destination_x,destination_y,distance_centre2destination,distance_other2destination)==1):
           ser.write('\xfd')
           break''' 
        
        #orientmove(destination_slope,bot_slope,Bot_position[0][0],Bot_position[0][1],destination_position[0][0],destination_position[0][1],distance_centre2destination,distance_other2destination)

        cv2.imshow('frame',frame)
        if cv2.waitKey(100)==27:
            break

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
#################################################
