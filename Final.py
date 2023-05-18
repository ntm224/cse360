import socket
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np
import math
import networkx as nx
import cv2
from robot.VideoStreaming import VideoStreaming

IP_ADDRESS = '192.168.0.206'

keypointsList = []

def camera_listener(image):
    frame2 = image
    hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    invert = cv2.bitwise_not(mask)
    blank = np.zeros((1,1))
    
    rows = mask.shape[0]
    
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    
    
    detector = cv2.SimpleBlobDetector.create(params)
    keypoints = detector.detect(invert)
    blobs = cv2.drawKeypoints(frame2, keypoints, blank, (0, 0, 255),
                                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    keypointsList =  keypoints

positions = {}
rotations = {}

def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

try:
    if __name__ == "__main__":
        clientAddress = "192.168.0.26"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 306
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        streaming_client.rigid_body_listener = receive_rigid_body_frame
        is_running = streaming_client.run()

        client = VideoStreaming()
        client.streaming(IP_ADDRESS, listener=camera_listener)
        client.connect()
        count = 0
        duck = 0

        while is_running:
            if robot_id in positions:
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                xpos = positions[robot_id][0]
                ypos = positions[robot_id][1]
                if(count == 0):
                    count = 1
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(700, 700, 700, 700)
                    s.send(command.encode('utf-8'))
                    time.sleep(4)
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 0, 0)
                    s.send(command.encode('utf-8'))
                    time.sleep(.5)
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(0, 0, 1500, 1500)
                    s.send(command.encode('utf-8'))
                    time.sleep(.5)
                else:
                
                    if(duck == 1):
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(0, 0, 1500, 1500)
                        s.send(command.encode('utf-8'))
                        time.sleep(4.5)
                        duck = 2
                    elif(duck == 2 and count <= 4):
                        newx = positions[robot_id][0]
                        newy = positions[robot_id][1]
                        if(positions[robot_id] - newx <= 0 and (positions[robot_id][1] < -1 or positions[robot_id][1] > 1)):
                            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 0, 0)
                            s.send(command.encode('utf-8'))
                            time.sleep(.5)
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(700, 700, 700, 700)
                        s.send(command.encode('utf-8'))
                        count = count + .1
                        time.sleep(.1)
                    elif(duck == 2 and count > 4 and count <= 10):
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1000, 1000, 1500, 1500)
                        s.send(command.encode('utf-8'))
                        count = count + .1
                        time.sleep(.1)
                    elif(duck == 2 and count > 10 and count <= 20):
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(0, 0, 1500, 1500)
                        s.send(command.encode('utf-8'))
                        time.sleep(1)
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 1000, 1000)
                        s.send(command.encode('utf-8'))
                        count = count + .1
                        time.sleep(.1)
                    elif(duck == 2 and count > 20):
                        count = 1
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-700, -700, -700, -700)
                        s.send(command.encode('utf-8'))
                        time.sleep(1)
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(0, 0, 1500, 1500)
                        s.send(command.encode('utf-8'))
                        time.sleep(1)
                        keypointsList = []

                    if((xpos >= 5.25 and xpos <= 5.75) and (ypos >= -.5 and ypos <= 0.5)):
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, -1500, -1500)
                        print("hi")
                        s.send(command.encode('utf-8'))
                        time.sleep(1.5)
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 0, 0)
                        s.send(command.encode('utf-8'))
                        time.sleep(3)
                        duck = 0
                    elif(xpos <= -4 or xpos >= 6 or ypos <= -2.5 or ypos >= 3.75):
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 0, 0)
                        s.send(command.encode('utf-8'))
                        time.sleep(3)
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1200, 1200, 1200, 1200)
                        s.send(command.encode('utf-8'))
                        time.sleep(.5)
                    elif(len(keypointsList) == 0 and duck == 0):
                        while(len(keypointsList) == 0):
                            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 0, 0)
                            s.send(command.encode('utf-8'))
                            time.sleep(1)
                            keypointsList = [150]
                        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1100, 1100, 1200, 1200)
                        s.send(command.encode('utf-8'))
                        time.sleep(3)
                    elif(duck == 0):
                        while(len(keypointsList) > 0):
                            if(keypointsList[0] < 250):
                                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(0, 0, 1500, 1500)
                                keypointsList = [450]
                                s.send(command.encode('utf-8'))
                                time.sleep(1)
                            elif(keypointsList[0] > 350):
                                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 0, 0)
                                keypointsList = [300]
                                s.send(command.encode('utf-8'))
                                time.sleep(1)
                            else: 
                                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1000, 1000, 1000, 1000)
                                s.send(command.encode('utf-8'))
                                time.sleep(2)
                                keypointsList = []
                                duck = 1

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    #client.StopTcpcClient()

# Close the connection
s.shutdown(2)
s.close()
#client.StopTcpcClient()
