import cv2
import numpy as np
import os
from time import sleep


#Define Constants

FRAME_RATE = 60

camera_matrix = np.array([[527.48423865, 0, 282.80328981],[0, 526.83629526, 213.53076347],[0, 0, 1]])

distortion_coeffs = np.array([[-0.08371482,  0.15038399, -0.00156261, -0.00139089, -0.03521561]])

h = 480
w = 640

optimalcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs,(w,h), 0, (w,h))

# Create ArUco detector object
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Create blob detector object
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByCircularity = False
params.filterByInertia = False
params.filterByConvexity = False

params.filterByArea = True # We will get a binary image with the puck thresholded, so only detect by area
params.minArea = 5 # Minimum area (in pixels) of a blob to be recognized as a puck

blob_detector = cv2.SimpleBlobDetector_create(params)

# Dimension of table and AruCo marker border in cm
table_dim_cm = (200,100)
aruco_border_cm = 1 # Should be unused, leaving it in as legacy
scaling_factor = 5 # Will be calculated later, leaving it in as legacy
machine_distance = 2 #cm from AI end

# HSV values to detect puck  # define range of red color in HSV
lower_puck = np.array([0,100,100])
upper_puck = np.array([10,255,255])

# Use table corners to perform perspective transform
rectangle_dim = (table_dim_cm[0] + 2 * aruco_border_cm + machine_distance, table_dim_cm[1] + 2 * aruco_border_cm) # This is the dimensions of the rectangle formed by the four ArUco marker corners.

output_length_pixels = scaling_factor * rectangle_dim[0] # Obtain the pixel dimensions of the perspective transformed image
output_width_pixels = scaling_factor * rectangle_dim[1]


#Defining functions

# Calibration function - requires an image as input and returns a calibration matrix M based on the current frame.
def calibration(image):
    print("Calibrating...")

    # If camera is flipped we can rotate the feed 180
    # image = cv2.rotate(image, cv2.ROTATE_180)

    # Undistort the image
    image = cv2.undistort(image, camera_matrix, distortion_coeffs, None, optimalcameramtx)

    # Detect for ArUco markers and save corner pixels
    (corners, ids, rejected) = detector.detectMarkers(image)

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
    rectangle_dim = (table_dim_cm[0] + 2 * aruco_border_cm + machine_distance, table_dim_cm[1] + 2 * aruco_border_cm) # This is the dimensions of the rectangle formed by the four ArUco marker corners.

    output_length_pixels = scaling_factor * rectangle_dim[0] # Obtain the pixel dimensions of the perspective transformed image
    output_width_pixels = scaling_factor * rectangle_dim[1]

    output_pts = np.float32([[aruco_border_cm*scaling_factor, aruco_border_cm*scaling_factor],
                              [output_length_pixels - (aruco_border_cm + machine_distance)*scaling_factor, aruco_border_cm*scaling_factor],
                              [aruco_border_cm*scaling_factor, output_width_pixels - aruco_border_cm*scaling_factor],
                              [output_length_pixels - (aruco_border_cm + machine_distance)*scaling_factor, output_width_pixels - aruco_border_cm*scaling_factor]])

    table_corners = np.float32(table_corners) # Must be a numpy array

    M = cv2.getPerspectiveTransform(table_corners,output_pts)
    #image = cv2.warpPerspective(image,M,(output_length_pixels, output_width_pixels),flags=cv2.INTER_LINEAR)
    #cv2.imshow(image)

    print("Calibration complete.")
    
    return M

# Puck Tracking function - Takes an image and the calibration matrix M as inputs and returns a tuple with (x,y) coordinates of the puck
def Track_puck(image, M):
    #print("Puck tracking...")

    # Undistort the image
    image = cv2.undistort(image, camera_matrix, distortion_coeffs, None, optimalcameramtx)

    # This is where all of the calibration happened in the last section, but we already have a matrix M so it can be skipped
    image = cv2.warpPerspective(image,M,(output_length_pixels, output_width_pixels),flags=cv2.INTER_LINEAR)

    # Turning the image to hsv format for masking/blob detection
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Mask it based on the upper and lower limits as defined in constants
    mask = cv2.inRange(hsv, lower_puck, upper_puck)  # Threshold the HSV image using inRange function to get only get the puck
    keypoints = blob_detector.detect(mask) # Detect blobs, whose coordinates are summarized in a keypoints object
    blobs = cv2.drawKeypoints(image, keypoints, (0,255,255), cv2.DRAW_MATCHES_FLAGS_DEFAULT)
    cv2.rectangle(blobs, (aruco_border_cm * scaling_factor, aruco_border_cm * scaling_factor),
              (output_length_pixels - (aruco_border_cm * scaling_factor), output_width_pixels - (aruco_border_cm * scaling_factor)), (0, 255, 0), 2) # Draw playing area rectangle

    #cv2.imshow(mask)
    #cv2.imshow(blobs)
    
    # Find location of the largest blob
    if len(keypoints) == 0:
        return -1

    size = [key_point.size for key_point in keypoints]
    index = max(range(len(size)), key=size.__getitem__)
    puckxy = keypoints[index].pt
    puckxy = tuple(map(int,puckxy))

    return puckxy




# Create videocapture object
vid = cv2.VideoCapture(1)
print("Opening PSeye")
vid.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('H','2','4','6'))
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
vid.set(cv2.CAP_PROP_FPS, FRAME_RATE)
if not vid.isOpened():
    print('VideoCapture not opened')
    exit(-1)
i = 0

print("Video Capture Opened")

# Show a preview until calibration begins
while True:
    ret, frame = vid.read()

    if not ret:
        print('frame empty')
        break

    cv2.imshow('Camera Feed', frame)

    # Press 'q' to break and start calibration
    if cv2.waitKey(1)&0XFF == ord('q'):
        break
    
# Calibration
M = calibration(frame)

# Write the pre- and post- calibration images to confirm it calibrated correctly
cv2.imwrite('Uncalibrated.png',frame)

# Undistort the image
image = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, optimalcameramtx)
# This is where all of the calibration happened in the last section, but we already have a matrix M so it can be skipped
image = cv2.warpPerspective(image,M,(output_length_pixels, output_width_pixels),flags=cv2.INTER_LINEAR)

cv2.imwrite('Calibrated.png',image)

# Show a preview again before beginning puck tracking/recording
while True:
    ret, frame = vid.read()

    if not ret:
        print('frame empty')
        break

    cv2.imshow('Camera Feed', frame)

    # Press 'q' to break and start puck tracking
    if cv2.waitKey(1)&0XFF == ord('q'):
        break

# To test/debug the puck tracking I will draw on a circle where the puck is supposed to be and record the frames as a video

out = cv2.VideoWriter('Tracking_test.avi',cv2.VideoWriter_fourcc('M','J','P','G'), FRAME_RATE, (output_length_pixels,output_width_pixels))
oldpuckxy = (0,0)

# Puck tracking and recording
print("Puck Tracking...")
while True:
    ret, frame = vid.read()

    if not ret:
        print('frame empty')
        break

    # Get the xy coordinates of the puck
    puckxy = Track_puck(frame,M)
    #print(puckxy)

    # If no puck is detected -1 is returned. That must be handled
    if puckxy == -1:
        puckxy = oldpuckxy
        print("Puck not detected")

    # Undistort and transform the image (just for the video)
    image = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, optimalcameramtx)
    image = cv2.warpPerspective(image,M,(output_length_pixels, output_width_pixels),flags=cv2.INTER_LINEAR)

    # Draw a circle on the puck in the image
    image = cv2.circle(image, puckxy, 2, (0,255,255),2)

    cv2.imshow('Processed image', image)
    out.write(image)

    # Press 'q' to break and finish recording
    if cv2.waitKey(1)&0XFF == ord('q'):
        break

print("Video complete. Closing...")
vid.release()
out.release()
cv2.destroyAllWindows()
