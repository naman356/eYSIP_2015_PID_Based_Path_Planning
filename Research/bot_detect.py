
############################################
## Import OpenCV
import numpy as np
import cv2
import heapq ##priority queue
import serial #library function for accessing xbee module
import math
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

last_route_length = 0
setpoint = 0
last_proportional = 0
integral = 0
angle = 0
c=0
Kp = 1
Ki = 0
Kd = 0

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
    cv2.circle(frame,(cx,cy), 2, (0,255,255), -1)

    destination_position[0][0] = cx
    destination_position[0][1] = cy

    #print cx,cy, "destination"

            
##########################################         
def bot_position(hsv):
    
    bot_lower = np.array([0,50,255])
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
    cv2.circle(frame,(cx1,cy1), 2, (255,0,255), -1)
    Bot_position[0][0]=cx1
    Bot_position[0][1]=cy1
    #print cx1,cy1
    #print Bot_position[0][0],
    M = cv2.moments(contours[1])
    cx2 = int(M['m10']/M['m00'])
    cy2 = int(M['m01']/M['m00'])
    cv2.circle(frame,(cx2,cy2), 2, (0,0,255), -1)
    Bot_position[1][0]=cx2
    Bot_position[1][1]=cy2

    #print cx1,cy1, "1"
    #print cx2,cy2, "2"
    
############################################
def wall_detection(hsv):
    lower = np.array([40 ,40, 240],np.uint8)
    upper = np.array([90, 255, 255],np.uint8)
    mask = cv2.inRange(hsv,lower, upper)
    #cv2.imshow('before fill maksed',mask)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours, key = cv2.contourArea, reverse = True)[:12]
    #contours,length=areacon(contours,3000,2100)
    #contours=sorted(contours, key = cv2.contourArea, reverse = True)[:length]
    cv2.fillPoly(mask,contours, (255,255,255))
    #cv2.imshow('maksed',mask)
    kernel = np.ones((50,50),np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    erosion = cv2.erode(mask,kernel,iterations = 1)
    dilation = cv2.dilate(closing,kernel,iterations = 1)#obstacle = cv2.dilate(closing,kernel,iterations = 1)
    cv2.imshow('obstacle',dilation)
    return dilation

##############################################
    
'''def grid_draw(frame,m,n): ##filename is image filename with full file path, n is grid of n lines
    for x in range(0, m): ##drawing lines
        X=x*line_widthm
        cv2.line(frame,(0,X),(k,X),(0,0,0), 2)#lines is red color, bgr format
    for y in range(0, n): ##drawing lines
        Y=y*line_widthn
        cv2.line(frame,(Y,0),(Y,h),(0,0,0), 2)
    return (frame)'''

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
         #print current
         if current == finish:
             name='Shortest Path, image#'
             i=int(100*np.random.rand())
             name=name+str(i)
             route=build_path(start, finish, link)
             ####Drawing path , just for pictorial representation######
             for i in range(1,len(route)):
                cv2.line(frame,(route[i-1].y*line_widthn+line_widthn/2,route[i-1].x*line_widthm+line_widthm/2),(route[i].y*line_widthn+line_widthn/2,route[i].x*line_widthm+line_widthm/2),(200,200,0), 3)
             cv2.imshow('path',frame)
             ############################
             return g[current], route[1:len(route)]
            
        
         moves = current.get_moves()
         cost = g[current]
         for mv in moves:
            #print mv.x,mv.y
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
    #cv2.imshow("walls in grid map",walls)
    #creating 10x10 matrix space map with black as obstable and other colors as paths.
    for x in range(0,grid_line_x-1):
        X=x*line_widthm+(line_widthm/2)
        for y in range(0,grid_line_y-1):
            Y=y*line_widthn+(line_widthn/2)
            if walls[X,Y]>=250 or x==0 or x==grid_line_x-2 or y==0 or y==grid_line_y-2: # and frame[X,Y,1]>=70 and frame[X,Y,2]>=70: #obstacle black ,bgr value(0,0,0)
                grid_map[x][y]=1
                cv2.circle(frame,(Y,X), 1, (0,50,200), -1)
                #cv2.imshow('frame',frame)
            continue
    last_grid_map = grid_map
    #print grid_map

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
        #print integral
        proportional = error - setpoint
        #integral = integral + proportional
        last_proportional = proportional

        derivative = (last_proportional) - (proportional)

        correction = Kp*proportional + Ki*integral + Kd*derivative

        return correction
    
##################################
def modified_angle(angle):
        angle = angle/10
        angle = angle*10
        return angle
    
############################    
def orientmove(slope_bot2cell,slope_botmarkers,bot_grid_x,bot_grid_y,route_x,route_y,distance_centre2cell,distance_other2cell):
        #print bot_grid_x,bot_grid_y,route_x,route_y
        #print stepper
        global c
        bot_grid_x, bot_grid_y = grid_to_pixel(bot_grid_x,bot_grid_y,line_widthm,line_widthn)
        route_x,route_y = grid_to_pixel(route_x,route_y,line_widthm,line_widthn)
        
        if bot_grid_x==route_x and bot_grid_y==route_y: #check if bot has reached next coordinate
                #ser.write("5")
                print "Hello"
                #ser.write("7")
                ser.write("S")
                
                #return 1
        else:
                
                if slope_bot2cell*slope_botmarkers!=-1 :
                        theta=math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))
                        angle = int(math.degrees(theta)) # print theta in degree
                        #angle = modified_angle(angle)
                        
                        pid_correction = pid_value(angle)
                        if(pid_correction < -255):
                            pid_correction = -255
                        if(pid_correction >255):
                            pid_correction = 255
                        c=c+1
                        print pid_correction,"no. of times",c
                          
                        if distance_other2cell>distance_centre2cell: #if other marker is farther from the grid cell's centre then move it closer with fast turns
                                 if theta<20: #20 for theta greater than 90 degrees, here theta is in radians
                                       ser.write("A")  #fast right turn
                                 else:
                                       ser.write("D")   #fast left turn
                        else:
                                 if(pid_correction == 0):
                                       ser.write("F")
                                 elif(pid_correction < 0):
                                       pid_correction = 255-(-1)*pid_correction
                                       print pid_correction,"R",hex(pid_correction)
                                       ser.write(hex(pid_correction))
                                       ser.write("R")
                                 else:
                                       pid_correction = 255-pid_correction
                                       print pid_correction,"L",hex(pid_correction)
                                       ser.write(hex(pid_correction))
                                       ser.write("L")
                                       #print pid_correction
                                
                                #ser.write(com) #send command
                        
                else:
                        return 0

##############################################
def bot_traverse(route_path,destination,frame):
    global stepper
    global current_path_following
    
    if stepper<=len(route_path)-1:
        X,Y=grid_to_pixel(route_path[stepper].x,route_path[stepper].y,line_widthm,line_widthn)#X,Y are pixels of next grid coor
        #bot = GridPoint((Bot_position[0][1]/line_widthn),(Bot_position[0][0]/line_widthm)) ##reversing coordinates so that it can be compatible with coordinate system of matrix
        rear_bot_x,rear_bot_y=get_coordinate(Bot_position[0][0],Bot_position[0][1])
        front_bot_x,front_bot_y=get_coordinate(Bot_position[1][0],Bot_position[1][1])

        d1 = dis(Bot_position[0][0],Bot_position[0][1],Y,X)
        d2 = dis(Bot_position[1][0],Bot_position[1][1],Y,X)

        mid_x=(Bot_position[0][0]+Bot_position[1][0])/2#mid point of bot center and other point
        mid_y=(Bot_position[0][1]+Bot_position[1][1])/2#mid point of bot center and other point
        bot_grid_x,bot_grid_y=get_coordinate(mid_x,mid_y)
        
        # print "dist",distance
        # print line_widthm,line_widthn,
        cv2.circle(frame,(Y,X), 5, (255,100,100), -1)
        # print "hello123"
        m1= getslope(Bot_position[0][0],Bot_position[0][1],Y,X)
        
        # print "slope m1", m1
        m2= getslope(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])

        
        '''print route_path[stepper].x,route_path[stepper].y,"route"
        print X,Y,"route_pixel"
        print m1,"next",m2,"bot"
        print len(route_path),"length"
        print d1,"bot_to_next",d2,"bot"
        print rear_bot_x,rear_bot_y,"rear"
        print front_bot_x,front_bot_y,"front"
        print route_path,destination
        #print "slope m2", m2
        #print "bot",rear_bot_x+1,rear_bot_y+1," next ",route_path[stepper].y+1,route_path[stepper].x+1'''
        if orientmove(m1,m2,bot_grid_x+1,bot_grid_y+1,route_path[stepper].y+1,route_path[stepper].x+1,d1,d2)==1: #bot reaches next coor
            stepper=stepper+1
            #flag=0
            #print "Bot Coor",rear_bot_x+1,rear_bot_y+1
            print stepper,"when reached next"
            ser.write("9")
            print bot_grid_x+1,bot_grid_y+1,"bot"
    
    else:
        #current_path_following=current_path_following+1
        #print "path no.",current_path_following
        #stepper=0
        ser.write("S")
        ser.write("9")
        print stepper
        print route_path[stepper-1].y+1,route_path[stepper-1].x+1
        print "reached"
        #break
        
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
    '''if last_grid_map != grid_map :
        route_path = last_route_path
    
        print route_length, route_path
        
    path_sample = path_sample - 1'''
    last_route_length = route_length
    last_route_path = route_path
    #print route_length, route_path
    #bot_traverse(route_path_list,grid_end,frame)
    #bot_traverse(route_path_list,grid_end,frame)

####################################################
if __name__ == "__main__":    
    cap=cv2.VideoCapture(1)
    ret,frame=cap.read()
    k=100
    global h,k,l
    global line_widthm,line_widthn
    first_frame = cv2.imwrite("pic_11.jpeg",frame)
    while k :
        k= k-1
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    h,k,l=frame.shape
    line_widthm=h/(grid_line_x-1)
    line_widthn=k/(grid_line_y-1)

    #bot_position(hsv)
    #destination(hsv)
                          
    #walls=wall_detection(hsv)
    #grid_map_of_walls(walls,grid_line_x,grid_line_y)
    #bot_route(frame)
    
    #cv2.imshow('first_frame',frame)
    while(1):
        ## Read the image
        #img = cv2.imread('output_image.jpg')
        ret,frame=cap.read()
        
        # moving_objects = frame - first_frame
        # cv2.imshow('moving',moving_objects)
        
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        bot_position(hsv)
        destination(hsv)

        h,k,l=frame.shape
        line_widthm=h/(grid_line_x-1)
        line_widthn=k/(grid_line_y-1)

        '''for x in range(0, grid_line_x): ##drawing lines
            X=x*line_widthm
            cv2.line(frame,(0,X),(k,X),(0,0,255), 1)#lines is red color, bgr format
        for y in range(0, grid_line_y): ##drawing lines
            Y=y*line_widthn
            cv2.line(frame,(Y,0),(Y,h),(255,0,0), 1)#lines is red color, bgr format
        # print h,k,l,"frame size"'''
        
        walls=wall_detection(hsv)
        grid_map_of_walls(walls,grid_line_x,grid_line_y)
        
        cv2.line(frame,(destination_position[0][0],destination_position[0][1]),(Bot_position[0][0],Bot_position[0][1]),(255,255,0), 1)
        cv2.line(frame,(Bot_position[0][0],Bot_position[0][1]),(Bot_position[1][0],Bot_position[1][1]),(0,255,255), 1)

        destination_slope = getslope(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        bot_slope = getslope(Bot_position[0][0],Bot_position[0][1],Bot_position[1][0],Bot_position[1][1])

        distance_centre2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[0][0],Bot_position[0][1])
        distance_other2destination = dis(destination_position[0][0],destination_position[0][1],Bot_position[1][0],Bot_position[1][1])

        destination_x,destination_y = get_coordinate(destination_position[0][0],destination_position[0][1])
        rear_bot_x,rear_bot_y=get_coordinate(Bot_position[0][0],Bot_position[0][1])
        
        # print bot_slope,"bot",destination_slope,"destination"
        #bot_route(frame)

        #bot_traverse(route_path,grid_end,frame)
        orientmove(destination_slope,bot_slope,rear_bot_x,rear_bot_y,destination_x,destination_y,distance_centre2destination,distance_other2destination)
        
        # print path_sample,"new"
        #orientmove(destination_slope,bot_slope,Bot_position[0][0],Bot_position[0][1],destination_position[0][0],destination_position[0][1],distance_centre2destination,distance_other2destination)
        cv2.imshow('frame',frame)
        if cv2.waitKey(100)==27:
            break

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
#################################################
