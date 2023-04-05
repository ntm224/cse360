import numpy as np
import cv2
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
from Motor import *            
PWM=Motor()
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
PWM.setMotorModel(0,0,0,0)

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

try:
    if __name__ == "__main__":
        clientAddress = "192.168.0.206"
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
        while is_running:
            frame2 = picam2.capture_array()

            hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            invert = cv2.bitwise_not(mask)
            blank = np.zeros((1,1))
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rows = mask.shape[0]
            
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea = False
            params.filterByCircularity = False
            params.filterByConvexity = False
            params.filterByInertia = False
            
            
            detector = cv2.SimpleBlobDetector.create(params)
                  
            # Detect blobs
            keypoints = detector.detect(invert)
              
            # Draw blobs on our image as red circles
            blobs = cv2.drawKeypoints(frame2, keypoints, blank, (0, 0, 255),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            if(len(keypoints) > 0):
                print("x: ", keypoints[0].pt[0], "\ny: ", keypoints[0].pt[1], "\nsize: ", keypoints[0].size)
                K = np.array([[1667, 0, 640], [0, 1667, 512], [0, 0, 1]])
                Ki = np.linalg.inv(K)
                r1 = Ki.dot([keypoints[0].pt[0], keypoints[0].pt[1], 1.0])
                r2 = Ki.dot([255, 255, 1.0])
                cos_angle = r1.dot(r2) / (np.linalg.norm(r1) * np.linalg.norm(r2))
                angle_radians = np.arccos(cos_angle)
                angle = np.rad2deg(angle_radians)
                print("angle: ", angle)

                z = (2 * 510)/keypoints[0].size
                print("distance: ", z)
                duck_location = z * np.sin(angle)
                print("location: ", duck_location)
            cv2.imshow("detected shapes", blobs)
            cv2.imshow("color", invert)

            time.sleep(.1)

except KeyboardInterrupt:
    # STOP
    PWM.setMotorModel(0,0,0,0)
    cv2.destroyAllWindows()

# When everything done, release the capture
cv2.destroyAllWindows()
PWM.setMotorModel(0,0,0,0)
