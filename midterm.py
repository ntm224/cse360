import socket
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np
import math

IP_ADDRESS = '192.168.0.206'

positions = {}
rotations = {}

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

try:
    if __name__ == "__main__":
        clientAddress = "192.168.0.26"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 206

        # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()

        timer = .0

        while (is_running):
            if robot_id in positions:
                # last position
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

                    
                if(timer <= .25):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= .75):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 1.25):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 1.75):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 2.25):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 2.75):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 3.25):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 3.75):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(-1500, -1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                elif(timer <= 4):
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(1500, 1500, 1500, 1500)
                    s.send(command.encode('utf-8'))
                else:
                    command = 'CMD_MOTOR#00#00#00#00\n'
                    s.send(command.encode('utf-8'))

                timer = timer + .01
                time.sleep(.01)

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

# Close the connection
s.shutdown(2)
s.close()
