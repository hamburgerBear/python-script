#-*- coding: utf-8 -*-

import matplotlib.pyplot as plt 
import matplotlib.image as mpimg 
import numpy as np
import cv2
import thread 
import time 
import socket

# MACRO DEFINITION
X = 0
Y = 1
R = 2
G = 1
B = 0

REA = [0x00, 0x00, 0xff]
WHITE = [0xff, 0xff, 0xff]
BLACK = [0x00, 0x00, 0x00]
GRAY = [0x80, 0x80, 0x80]
GREEN = [0x00, 0xff, 0x00]
PURPLE = [0xff, 0x00, 0xff]
ORANGE = [0x00, 0x66, 0xff]
BLUE = [0xFF, 0x00, 0x00]

INDEX_HEAD1 = 0
INDEX_HEAD2 = 1
INDEX_CMD   = 2
INDEX_LENL  = 3
INDEX_LENH  = 4
INDEX_ID    = 5
INDEX_DATA  = 6

GRIDMAP_CMD = 0x86
ROBOTPOSE_CMD = 0x89
AIMPOINT_CMD = 0x91
WAYPOINT_CMD = 0x94

SIZE_X = 400
SIZE_Y = 400

# GLOBAL VARIABLES
grid_map = np.zeros([400, 400, 3], np.uint8)
show_map = np.zeros([400, 400, 3], np.uint8)
package_id = 0
package_len = 0
robot_pose = [0, 0]
aimpoint = np.zeros([100, 2], np.int)
aimpoint_len = 0
waypoint = np.zeros([100, 2], np.int)
waypoint_len = 0
updateWindow = False
pathpoint = np.zeros([200, 2], np.int)
pathpoint_len = 0

def udpClient(threadName, delay):
    global show_map
    global grid_map
    global robot_pose
    global updateWindow
    global aimpoint    
    global waypoint
    global aimpoint_len
    global waypoint_len

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip_port = ('192.168.100.1', 8000)
    client.sendto('connect'.encode('utf-8'), ip_port)

    while True:
        data = client.recv(1024) # receive data from server 

        if ord(data[INDEX_CMD]) == GRIDMAP_CMD:

	    package_id = ord(data[INDEX_ID])
	    package_len = ord(data[INDEX_LENL]) + (ord(data[INDEX_LENH]) << 8)

	    for i in range(package_len):
	    
	        if ord(data[INDEX_DATA+i]) == 1:            # clean refrence white color
	        
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][R] = WHITE[R]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][G] = WHITE[G]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][B] = WHITE[B]

		
	        elif ord(data[INDEX_DATA+i]) == 240:        # obstacle refrence black color
	            grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][R] = BLACK[R]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][G] = BLACK[G]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][B] = BLACK[B]
	
	    
	        elif ord(data[INDEX_DATA+i]) == 255:        # default gray eare gray color
	            grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][R] = GRAY[R]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][G] = GRAY[G]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][B] = GRAY[B]	        
		
	    
	        elif ord(data[INDEX_DATA+i]) == 251:        # along wall refrence green color 
	            grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][R] = GREEN[R]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][G] = GREEN[G]
		    grid_map[int((package_id * package_len + i)/SIZE_X)][(package_id * package_len + i)%SIZE_X][B] = GREEN[B]
	        
	        else:
		    pass
        
	    if package_id == 159:
		updateWindow = True

	elif ord(data[INDEX_CMD]) == ROBOTPOSE_CMD:   
	    robot_pose[X] = ord(data[INDEX_DATA+0]) + (ord(data[INDEX_DATA+1]) << 8)
	    robot_pose[Y] = ord(data[INDEX_DATA+2]) + (ord(data[INDEX_DATA+3]) << 8)

	elif ord(data[INDEX_CMD]) == AIMPOINT_CMD: 
	    aimpoint_len = ord(data[INDEX_LENL]) + (ord(data[INDEX_LENH]) << 8)
     	    for i in range(aimpoint_len):
                aimpoint[i][X] =  ord(data[6+i*4+0]) + (ord(data[6+i*4+1]) << 8)
		aimpoint[i][Y] =  ord(data[6+i*4+2]) + (ord(data[6+i*4+3]) << 8) 
              	
	elif ord(data[INDEX_CMD]) == WAYPOINT_CMD: 
	    waypoint_len = ord(data[INDEX_LENL]) + (ord(data[INDEX_LENH]) << 8)
     	    for i in range(waypoint_len):
                waypoint[i][X] =  ord(data[6+i*4+0]) + (ord(data[6+i*4+1]) << 8)
		waypoint[i][Y] =  ord(data[6+i*4+2]) + (ord(data[6+i*4+3]) << 8)

	else:
	    pass



def showWindow(threadName, delay):
    global show_map
    global grid_map
    global robot_pose
    global updateWindow
    global aimpoint    
    global waypoint
    global aimpoint_len
    global waypoint_len

    while True:

	if updateWindow == True:
	    updateWindow = False
            show_map = grid_map

	for i in range(aimpoint_len):
	    show_map[aimpoint[i][X]][aimpoint[i][Y]][R] = PURPLE[R]
            show_map[aimpoint[i][X]][aimpoint[i][Y]][G] = PURPLE[G]
	    show_map[aimpoint[i][X]][aimpoint[i][Y]][B] = PURPLE[B]

	for i in range(waypoint_len):
	    show_map[waypoint[i][X]][waypoint[i][Y]][R] = ORANGE[R]
            show_map[waypoint[i][X]][waypoint[i][Y]][G] = ORANGE[G]
	    show_map[waypoint[i][X]][waypoint[i][Y]][B] = ORANGE[B]

	cv2.circle(show_map, (robot_pose[Y], robot_pose[X]), 1, (255,0,0))
	show_map[robot_pose[X]][robot_pose[Y]][R] = REA[R]
	show_map[robot_pose[X]][robot_pose[Y]][G] = REA[G]
	show_map[robot_pose[X]][robot_pose[Y]][B] = REA[B]

	cv2.putText(show_map, 'x:'+str(robot_pose[X])+' y:'+str(robot_pose[Y]), (20,380), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,125,255), 2)	

	cv2.namedWindow('IMG', cv2.WINDOW_NORMAL)
	cv2.imshow('IMG', show_map)
	cv2.waitKey(10)

thread.start_new_thread(udpClient, ('udpClient', 10))  
thread.start_new_thread(showWindow, ('showWindow', 10))  
 
while True:
    pass
    


