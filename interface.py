import time
import numpy as np
import os
import cv2
import serial
import sys
from datetime import datetime

class PuckTracker():
    def __init__(self, com_port="COM5"):
        self.start_time = time.time()
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.com_port = com_port

        self._create_blob_detector()
        self._establish_serial()
        # self._create_logger()

        self.XMIN = 0.1
        self.XMAX = 0.9
        self.YMIN = 0.1
        self.YMAX = 1.9 
        
        self.puck_pos = [-100, -100]
        self.puck_vel = [0.0, 0.0]
        
        # Make the image 200x400, two pixels per cm
        self.des_image_shape = (200, 400)
        self.pixels_to_cm = 0.5
        # Use get_corners.py to get from_corners
        self.from_corners = [[11,71],[604,57],[633,332],[13,368]]
        self.to_corners = [[0,400],[0,0],[200,0],[200,400]]

        self.transform_matrix = cv2.getPerspectiveTransform(np.float32(self.from_corners), np.float32(self.to_corners))

        print('Run the bash script on the Pi now')
        self.vid = cv2.VideoCapture('udp://0.0.0.0:10000?overrun_nonfatal=1&fifo_size=5000000')
        # print("about to readall")
        self.serial.read_all()
        self.serial.read_until("\n")
        print('Connected to Pi')
        self.vid.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('H','2','6','4'))

        self.frame = self.vid.read()[1]
        self.w = self.frame.shape[0]
        self.h = self.frame.shape[1]
        
        self.last_frame_time = time.time()  # seconds

        # Best parameters found
        self.camera_matrix = np.array([[283.08720786,   0.        , 319.49999987],
            [  0.        , 224.13115655, 239.49999971],
            [  0.        ,   0.        ,   1.        ]])

        self.distortion_coeffs = np.array([[-1.55053177e-02,  5.16288067e-05, -5.41872511e-03,
        -2.47583796e-03, -4.58942756e-08]])

        self.optimalcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_coeffs, (self.w,self.h), 0, (self.w,self.h))

        # self.start_motion()


    def _create_blob_detector(self):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 300
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.7
        
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        
        self.detector = cv2.SimpleBlobDetector_create(params)


    def _establish_serial(self):
        self.serial = serial.Serial(self.com_port, 460800, timeout=0.01)
        self.serial.read_all()

    
    def _create_logger(self):
        file_name = self.dir_path + '/data/' + datetime.now().strftime("%Y%m%d_%H%M%S.csv")
        self.logger = open(file_name, "w")
        # self.logger.write("BEGIN CSV\n")


    def start_motion(self):
        self.serial.write('s\n'.encode())


    def send_to_bluepill(self):
        msg = f"{round(self.frame_time,4)},{round(self.puck_pos[0],4)},{round(self.puck_pos[1],4)},{round(self.puck_vel[0],4)},{round(self.puck_vel[1],4)}\n"
        print('Writing to Blue Pill')
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
        return self.puck_pos[0] >= self.XMIN and self.puck_pos[0] <= self.XMAX \
            and self.puck_pos[1] >= self.YMIN and self.puck_pos[1] <= self.YMAX

    def update_puck_status(self):
        frame = self.vid.read()[1]
        self.frame_time = 1000* (time.time() - self.start_time)

        self.frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coeffs, None, self.optimalcameramtx)
        # cv2.imshow('Distortion Corrected',self.frame)

        self.frame = cv2.warpPerspective(self.frame, M=self.transform_matrix, dsize=self.des_image_shape)

        keypoints = self.detector.detect(self.frame)
        # print(keypoints)

        if len(keypoints):
            x = keypoints[0].pt[0]
            y = keypoints[0].pt[1]
            # im_with_keypoints = cv2.drawKeypoints(self.frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # cv2.imshow("Keypoints", im_with_keypoints)

            if x > -1 and y > -1 and self.puck_pos[0] > -1 and self.puck_pos[1] > -1:
                vx = (x - self.puck_pos[0]) / (self.frame_time - self.last_frame_time)
                vy = (y - self.puck_pos[1]) / (self.frame_time - self.last_frame_time)

                epsilon = 0.6 #higher is mostly current [0; 1]
                self.puck_vel[0] = (1-epsilon)*self.puck_vel[0] + epsilon*vx
                self.puck_vel[1] = (1-epsilon)*self.puck_vel[1] + epsilon*vy
            else:
                self.puck_vel = [0, 0]

            self.puck_pos[0] = keypoints[0].pt[0] / 200
            self.puck_pos[1] = 2 - keypoints[0].pt[1] / 200

        else:
            self.puck_pos = [-100, -100]
            self.puck_vel = [0, 0]
        
        # cv2.imshow('Frame',self.frame)
        # cv2.waitKey(1)
        
        print(f"{self.puck_pos[0]}\t{self.puck_pos[1]}")
        
        if self.should_send_msg():
            self.send_to_bluepill()
        
        self.read_from_bluepill()
        self.last_frame_time = self.frame_time
    
    def destroy(self):
        self.vid.release()
        self.read_from_bluepill()
        self.logger.close()
        self.serial.close()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        com_port = sys.argv[1]    
        puck_tracker = PuckTracker(com_port)
    else:
        puck_tracker = PuckTracker()

    try:
        while(True):
            puck_tracker.update_puck_status()
    except KeyboardInterrupt:
        puck_tracker.destroy()