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
        clientAddress = "192.168.0.43"
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

        x0 = -1
        y0 = -1

        xf1 = 2
        xf2 = 2
        xf3 = -2
        xf4 = -2
        yf1 = 2
        yf2 = -2
        yf3 = -2
        yf4 = 2

        timer = 0

        while (is_running):
            if robot_id in positions:
                # last position
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

                if(timer <= 5):
                    Ax = (((xf1-x0)/5)*timer) + x0
                    Ay = (((yf1-y0)/5)*timer) + y0

                    angle = math.degrees(math.atan2(Ay, Ax)) - rotations[robot_id]
                    omega = 6*math.degrees(math.atan2(math.sin(angle),math.cos(angle)))

                    v = 600*math.sqrt((Ax)**2 + (Ay)**2)
                    u = np.array([(v-omega), (v+omega)])
                elif timer <= 8:
                    u = np.array([-1500, 1500])
                elif timer <= 13:
                    Ax = (((xf2-xf1)/5)*timer) + xf1
                    Ay = (((yf2-yf1)/5)*timer) + yf1

                    angle = math.degrees(math.atan2(Ay, Ax)) - rotations[robot_id]
                    omega = 6*math.degrees(math.atan2(math.sin(angle),math.cos(angle)))

                    v = 600*math.sqrt((Ax)**2 + (Ay)**2)
                    u = np.array([(v-omega), (v+omega)])
                elif timer <= 21:
                    u = np.array([-1500, 1500])
                elif timer <= 26:
                    Ax = (((xf3-xf2)/5)*timer) + xf2
                    Ay = (((yf3-yf2)/5)*timer) + yf2

                    angle = math.degrees(math.atan2(Ay, Ax)) - rotations[robot_id]
                    omega = 6*math.degrees(math.atan2(math.sin(angle),math.cos(angle)))

                    v = 600*math.sqrt((Ax)**2 + (Ay)**2)
                    u = np.array([(v-omega), (v+omega)])
                elif timer <= 29:
                    u = np.array([-1500, 1500])
                elif timer <= 34:
                    Ax = (((xf4-xf3)/5)*timer) + xf3
                    Ay = (((yf4-yf3)/5)*timer) + yf3

                    angle = math.degrees(math.atan2(Ay, Ax)) - rotations[robot_id]
                    omega = 6*math.degrees(math.atan2(math.sin(angle),math.cos(angle)))

                    v = 600*math.sqrt((Ax)**2 + (Ay)**2)
                    u = np.array([(v-omega), (v+omega)])

                u[u > 1500] = 1500
                u[u < -1500] = -1500
                # Send control input to the motors
                print('motors: ', u[0], u[1])
                print(Ax, '   ', Ay)
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))

                timer = timer + .1
                time.sleep(.1)

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

# Close the connection
s.shutdown(2)
s.close()
