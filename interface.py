import time
import numpy as np
import os
import cv2
import serial
import sys
from time import sleep
from datetime import datetime

class PuckTracker():
    def __init__(self, cameraIndex=1):
        #Define Constants

        self.FRAME_RATE = 60

        self.camera_matrix = np.array([[527.48423865, 0, 282.80328981],[0, 526.83629526, 213.53076347],[0, 0, 1]])

        self.distortion_coeffs = np.array([[-0.08371482,  0.15038399, -0.00156261, -0.00139089, -0.03521561]])

        h = 480
        w = 640

        self.optimalcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_coeffs,(w,h), 0, (w,h))

        # Create ArUco detector object
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Create blob detector object
        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = False
        params.filterByCircularity = False
        params.filterByInertia = False
        params.filterByConvexity = False

        params.filterByArea = True # We will get a binary image with the puck thresholded, so only detect by area
        params.minArea = 5 # Minimum area (in pixels) of a blob to be recognized as a puck

        self.blob_detector = cv2.SimpleBlobDetector_create(params)

        # Dimension of table and AruCo marker border in cm
        self.table_dim_cm = (200,100)
        self.aruco_border_cm = 1 # Should be unused, leaving it in as legacy
        self.scaling_factor = 5 # Will be calculated later, leaving it in as legacy
        self.machine_distance = 2 #cm from AI end

        # HSV values to detect puck  # define range of red color in HSV
        self.lower_puck = np.array([0,100,100])
        self.upper_puck = np.array([20,255,255])

        # Use table corners to perform perspective transform
        self.rectangle_dim = (self.table_dim_cm[0] + 2 * self.aruco_border_cm + self.machine_distance, self.table_dim_cm[1] + 2 * self.aruco_border_cm) # This is the dimensions of the rectangle formed by the four ArUco marker corners.

        self.output_length_pixels = self.scaling_factor * self.rectangle_dim[0] # Obtain the pixel dimensions of the perspective transformed image
        self.output_width_pixels = self.scaling_factor * self.rectangle_dim[1]

        # Create videocapture object
        self.vid = cv2.VideoCapture(cameraIndex)
        print("Opening PSeye")
        self.vid.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('H','2','4','6'))
        self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.vid.set(cv2.CAP_PROP_FPS, self.FRAME_RATE)
        if not self.vid.isOpened():
            print('VideoCapture not opened')
            exit(-1)

        print("Video Capture Opened")

        # Show a preview until calibration begins
        while True:
            ret, frame = self.vid.read()

            if not ret:
                print('frame empty')
                break

            cv2.imshow('Camera Feed', frame)

            # Press 'q' to break and start calibration
            if cv2.waitKey(1)&0XFF == ord('q'):
                break
                
        # Calibration
        self.M = self.calibration(frame)

        # Write the pre- and post- calibration images to confirm it calibrated correctly
        cv2.imwrite('Uncalibrated.png',frame)

        # Undistort the image
        image = cv2.undistort(frame, self.camera_matrix, self.distortion_coeffs, None, self.optimalcameramtx)
        # This is where all of the calibration happened in the last section, but we already have a matrix M so it can be skipped
        image = cv2.warpPerspective(image,self.M,(self.output_length_pixels, self.output_width_pixels),flags=cv2.INTER_LINEAR)

        cv2.imwrite('Calibrated.png',image)

        # Show a preview again before beginning puck tracking/recording
        while True:
            ret, frame = self.vid.read()

            if not ret:
                print('frame empty')
                break

            cv2.imshow('Camera Feed', frame)

            # Press 'q' to break and start puck tracking
            if cv2.waitKey(1)&0XFF == ord('q'):
                break


    #Defining functions

    # Calibration function - requires an image as input and returns a calibration matrix M based on the current frame.
    def calibration(self, image):
        print("Calibrating...")

        # If camera is flipped we can rotate the feed 180
        # image = cv2.rotate(image, cv2.ROTATE_180)

        # Undistort the image
        image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs, None, self.optimalcameramtx)

        # Detect for ArUco markers and save corner pixels
        (corners, ids, rejected) = self.detector.detectMarkers(image)

        table_corners = [[],[],[],[]]
        #print(ids)

        for i in range(len(ids)):
            id = int(ids[i])
            corner_points = corners[i]

            if id == 1:
                corner_point = corner_points[0][2] # for ArUco ID = 1, we want the 3rd (index 2) corner
                table_corners[0] = corner_point #- ((-1*aruco_border_cm)*scaling_factor, (-1*aruco_border_cm*scaling_factor))
            elif id == 2:
                corner_point = corner_points[0][3] # for ArUco ID = 2, we want the 4th (index 3) corner
                table_corners[2] = corner_point #- ((-1*aruco_border_cm)*scaling_factor, (1*aruco_border_cm*scaling_factor))
            elif id == 3:
                corner_point = corner_points[0][0] # for ArUco ID = 3, we want the 1st (index 0) corner
                table_corners[3] = corner_point #- ((1*aruco_border_cm + machine_distance)*scaling_factor, (1*aruco_border_cm*scaling_factor))
            elif id == 4:
                corner_point = corner_points[0][1] # for ArUco ID = 3, we want the 2nd (index 1) corner
                table_corners[1] = corner_point #- ((1*aruco_border_cm + machine_distance)*scaling_factor, (-1*aruco_border_cm*scaling_factor))

        # Use table corners to perform perspective transform
        rectangle_dim = (self.table_dim_cm[0] + 2 * self.aruco_border_cm + self.machine_distance, self.table_dim_cm[1] + 2 * self.aruco_border_cm) # This is the dimensions of the rectangle formed by the four ArUco marker corners.

        output_length_pixels = self.scaling_factor * rectangle_dim[0] # Obtain the pixel dimensions of the perspective transformed image
        output_width_pixels = self.scaling_factor * rectangle_dim[1]

        output_pts = np.float32([[self.aruco_border_cm*self.scaling_factor, self.aruco_border_cm*self.scaling_factor],
                                [output_length_pixels - (self.aruco_border_cm + self.machine_distance)*self.scaling_factor, self.aruco_border_cm*self.scaling_factor],
                                [self.aruco_border_cm*self.scaling_factor, output_width_pixels - self.aruco_border_cm*self.scaling_factor],
                                [output_length_pixels - (self.aruco_border_cm + self.machine_distance)*self.scaling_factor, output_width_pixels - self.aruco_border_cm*self.scaling_factor]])

        table_corners = np.float32(table_corners) # Must be a numpy array

        M = cv2.getPerspectiveTransform(table_corners,output_pts)
        #image = cv2.warpPerspective(image,M,(output_length_pixels, output_width_pixels),flags=cv2.INTER_LINEAR)
        #cv2.imshow(image)

        print("Calibration complete.")
        
        return M
    
    def getPuckCoords(self):
        ret, frame = self.vid.read()

        if not ret:
            print('frame empty')
            return -1
        return self.__Track_puck(frame, self.M)

    # Puck Tracking function - Takes an image and the calibration matrix M as inputs and returns a tuple with (x,y) coordinates of the puck
    def __Track_puck(self, image, M):
        #print("Puck tracking...")

        # Undistort the image
        image = cv2.undistort(image, self.camera_matrix, self.distortion_coeffs, None, self.optimalcameramtx)

        # This is where all of the calibration happened in the last section, but we already have a matrix M so it can be skipped
        image = cv2.warpPerspective(image,M,(self.output_length_pixels, self.output_width_pixels),flags=cv2.INTER_LINEAR)

        # Turning the image to hsv format for masking/blob detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Mask it based on the upper and lower limits as defined in constants
        mask = cv2.inRange(hsv, self.lower_puck, self.upper_puck)  # Threshold the HSV image using inRange function to get only get the puck
        keypoints = self.blob_detector.detect(mask) # Detect blobs, whose coordinates are summarized in a keypoints object
        blobs = cv2.drawKeypoints(image, keypoints, (0,255,255), cv2.DRAW_MATCHES_FLAGS_DEFAULT)
        cv2.rectangle(blobs, (self.aruco_border_cm * self.scaling_factor, self.aruco_border_cm * self.scaling_factor),
                (self.output_length_pixels - (self.aruco_border_cm * self.scaling_factor), self.output_width_pixels - (self.aruco_border_cm * self.scaling_factor)), (0, 255, 0), 2) # Draw playing area rectangle

        #cv2.imshow(mask)
        #cv2.imshow(blobs)
        
        # Find location of the largest blob
        if len(keypoints) == 0:
            return -1

        size = [key_point.size for key_point in keypoints]
        index = max(range(len(size)), key=size.__getitem__)
        puckyx = keypoints[index].pt
        puckyx = tuple(float(elem)/(self.scaling_factor*100) for elem in puckyx)

        # Rotate the coordinate system by 180
        puckyx = tuple(map(lambda i, j: i/100 - j, self.table_dim_cm, puckyx))

        return puckyx
    
    def destroy(self):
        self.vid.release()



class AirHockeyAgent():
    def __init__(self, com_port="COM5"):
        self.start_time = time.time()
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.com_port = com_port

        self._establish_serial()
        self._create_logger()

        self.running = False

        self.XMIN = 0.1
        self.XMAX = 0.9
        self.YMIN = 0.1
        self.YMAX = 1.9 
        
        self.x_pos = -1
        self.y_pos = -1
        self.puck_vel = [0.0, 0.0]
        
        # # Make the image 200x400, two pixels per cm
        # self.des_image_shape = (200, 400)
        # self.pixels_to_cm = 0.5
        # # Use get_corners.py to get from_corners
        # self.from_corners = [[11,71],[604,57],[633,332],[13,368]]
        # self.to_corners = [[0,400],[0,0],[200,0],[200,400]]

        # self.transform_matrix = cv2.getPerspectiveTransform(np.float32(self.from_corners), np.float32(self.to_corners))

        print('Run the bash script on the Pi now')
        # self.vid = cv2.VideoCapture('udp://0.0.0.0:10000?overrun_nonfatal=1&fifo_size=5000000')

        # Initialize puck tracker
        self.puck_Tracker = PuckTracker(1)
        # print("about to readall")
        self.serial.read_all()
        self.serial.read_until("\n")

        # self.start_motion()



    def _establish_serial(self):
        self.serial = serial.Serial(self.com_port, 460800, timeout=0.01)
        self.serial.read_all()

    
    def _create_logger(self):
        file_name = self.dir_path + '/data/2024-03-20/Mimic/' + datetime.now().strftime("%Y%m%d_time%H%M%S.csv")
        self.logger = open(file_name, "w")
        # self.logger.write("BEGIN CSV\n")


    def start_motion(self):
        self.serial.write('s\n'.encode())


    def send_to_bluepill(self):
        msg = f"{round(self.frame_time,4)},{round(self.x_pos,4)},{round(self.y_pos,4)},{round(0.0,4)},{round(0.0,4)}\n"
        print("Sending: " + msg)
        self.serial.write(msg.encode())

    
    def read_from_bluepill(self):
        print('Reading from Blue Pill')
        data = self.serial.read_all()
        try:
            data = data.decode().replace('\r\n','\n')
            # print(data)
            if len(data):
                self.logger.write(data)
        except Exception as e:
            print(e)
            print('Could not decode data: ', data)
            return
        
        if data == "Finished executing\n":
            self.logger.close()
            exit(0)

    
    def should_send_msg(self):
        '''Decide whether to send a command to the Blue Pill'''
        return self.x_pos >= self.XMIN and self.x_pos <= self.XMAX \
            and self.y_pos >= self.YMIN and self.y_pos <= self.YMAX

    def update_puck_status(self):
        puckyx = self.puck_Tracker.getPuckCoords()
        self.frame_time = 1000* (time.time() - self.start_time)
        if puckyx != -1:
            self.x_pos = puckyx[1]
            self.y_pos = puckyx[0]
        # cv2.imshow('Frame',self.frame)
        # cv2.waitKey(1)
        
        print(f"{self.x_pos}\t{self.y_pos}")
        
        if self.should_send_msg() and self.running:
            self.send_to_bluepill()
        
        self.read_from_bluepill()
        self.last_frame_time = self.frame_time
        # print(puckyx)
    
    def destroy(self):
        self.puck_Tracker.destroy()
        # self.read_from_bluepill()
        # self.logger.close()
        self.serial.close()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        com_port = sys.argv[1]    
        agent = AirHockeyAgent(com_port)
    else:
        agent = AirHockeyAgent()

    try:
        while(True):
            agent.update_puck_status()
            # Press 'q' to break
            if cv2.waitKey(1)&0XFF == ord('q'):
                agent.running = not agent.running
    except KeyboardInterrupt:
        agent.destroy()