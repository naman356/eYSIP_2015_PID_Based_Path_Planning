############################################
## Import OpenCV
import numpy as np
import cv2
import heapq ##priority queue
import serial #library function for accessing xbee module
import math

h,k,l=frame.shape
line_widthm=h/(grid_line_x-1)
line_widthn=k/(grid_line_y-1)

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
        
