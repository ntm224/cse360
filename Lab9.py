import socket
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np
import math
import networkx as nx

IP_ADDRESS = '192.168.0.206'

positions = {}
rotations = {}

def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

G = nx.Graph()
points = []

pA = (1,1)
pB = (1,1.5)
pC = (1.5,1)
pD = (1.25,1.5)
pE = (1.5,1.25)
pF = (1.5,1.5)
pG = (2,1.5)
pH = (2.1,1.35)
pI = (1.8,1.3)
pJ = (2.2,1)
points.append(pA)
points.append(pB)
points.append(pC)
points.append(pD)
points.append(pE)
points.append(pF)
points.append(pG)
points.append(pH)
points.append(pI)
points.append(pJ)

G.add_edge("1", "2", weight=np.absolute(np.sqrt((pB[0]-pA[0])**2) + (pB[1]-pA[1])**2))
G.add_edge("1", "3", weight=np.absolute(np.sqrt((pC[0]-pA[0])**2) + (pC[1]-pA[1])**2))
G.add_edge("2", "4", weight=np.absolute(np.sqrt((pD[0]-pB[0])**2) + (pD[1]-pB[1])**2))
G.add_edge("3", "5", weight=np.absolute(np.sqrt((pE[0]-pC[0])**2) + (pE[1]-pC[1])**2))
G.add_edge("4", "6", weight=np.absolute(np.sqrt((pF[0]-pD[0])**2) + (pF[1]-pD[1])**2))
G.add_edge("5", "6", weight=np.absolute(np.sqrt((pF[0]-pE[0])**2) + (pF[1]-pE[1])**2))
G.add_edge("6", "7", weight=np.absolute(np.sqrt((pG[0]-pF[0])**2) + (pG[1]-pF[1])**2))
G.add_edge("6", "9", weight=np.absolute(np.sqrt((pI[0]-pF[0])**2) + (pI[1]-pF[1])**2))
G.add_edge("5", "9", weight=np.absolute(np.sqrt((pI[0]-pE[0])**2) + (pI[1]-pE[1])**2))
G.add_edge("9", "7", weight=np.absolute(np.sqrt((pG[0]-pI[0])**2) + (pG[1]-pI[1])**2))
G.add_edge("9", "8", weight=np.absolute(np.sqrt((pH[0]-pI[0])**2) + (pH[1]-pI[1])**2))
G.add_edge("7", "8", weight=np.absolute(np.sqrt((pH[0]-pG[0])**2) + (pH[1]-pG[1])**2))
G.add_edge("8", "10", weight=np.absolute(np.sqrt((pJ[0]-pH[0])**2) + (pJ[1]-pH[1])**2))
path = nx.shortest_path(G, "1", "10", weight="weight")

try:
    if __name__ == "__main__":
        clientAddress = "192.168.0.187"
        optitrackServerAddress = "192.168.0.172"
        robot_id = 206
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        streaming_client.rigid_body_listener = receive_rigid_body_frame
        is_running = streaming_client.run()
        count = 0

        for i in path:
            if(count == 0):
                count = 1
            else:
                while (is_running):
                    if robot_id in positions:
                        print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

                        while(np.absolute(math.degrees(math.atan2(points[int(i)][0], points[int(i)][1])) - rotations[robot_id]) > 10):
                            print(math.degrees(math.atan2(points[int(i)][0], points[int(i)][1])))
                            print(rotations[robot_id])

                            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1000, 1000, -1000, -1000)
                            s.send(command.encode('utf-8'))
                            time.sleep(.1)


                        while(np.absolute(np.sqrt((points[int(i)][0] - positions[robot_id][0])**2 + (points[int(i)][1] - positions[robot_id][1])**2))) > 0.05:
                            Ax = points[int(i)][0] - positions[robot_id][0]
                            Ay = points[int(i)][1] - positions[robot_id][1]
                            v = 2000*np.absolute(np.sqrt((Ax)**2 + (Ay)**2))
                            print(v)
                            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(v, v, v, v)
                            s.send(command.encode('utf-8'))
                            time.sleep(.1)

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

# Close the connection
s.shutdown(2)
s.close()
