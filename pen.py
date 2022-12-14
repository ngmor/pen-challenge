#!/usr/bin/env python3
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import enum
from tracemalloc import start
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import copy
from robot import Robot
import csv
import time

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
profile_stream = profile.get_stream(rs.stream.color)
intrinsics = profile_stream.as_video_stream_profile().get_intrinsics()

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Define HSV color limits
lower_H = 119#117
lower_S = 74#110
lower_V = 0#0

# B = 69, G = 27, R = 26
# H = 119, S = 159, V = 69
upper_H = 164#139
upper_S = 255
upper_V = 255

useTrackbars = False

window_name = 'Pen Tracker'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

if useTrackbars:
    lower_H_name = 'H Low'
    def trackbar_lower_H(val):
        global lower_H
        global upper_H
        lower_H = val
        lower_H = min(upper_H-1, lower_H)
        cv2.setTrackbarPos(lower_H_name,window_name,lower_H)
    cv2.createTrackbar(lower_H_name, window_name,lower_H,180,trackbar_lower_H)

    lower_S_name = 'S low'
    def trackbar_lower_S(val):
        global lower_S
        global upper_S
        lower_S = val
        lower_S = min(upper_S-1, lower_S)
        cv2.setTrackbarPos(lower_S_name,window_name,lower_S)
    cv2.createTrackbar(lower_S_name, window_name,lower_S,255,trackbar_lower_S)

    lower_V_name = 'V low'
    def trackbar_lower_V(val):
        global lower_V
        global upper_V
        lower_V = val
        lower_V = min(upper_V-1, lower_V)
        cv2.setTrackbarPos(lower_V_name,window_name,lower_V)
    cv2.createTrackbar(lower_V_name, window_name,lower_V,255,trackbar_lower_V)

    upper_H_name = 'H Upp'
    def trackbar_upper_H(val):
        global lower_H
        global upper_H
        upper_H = val
        upper_H = max(lower_H-1, upper_H)
        cv2.setTrackbarPos(upper_H_name,window_name,upper_H)
    cv2.createTrackbar(upper_H_name, window_name,upper_H,180,trackbar_upper_H)

    upper_S_name = 'S Upp'
    def trackbar_upper_S(val):
        global lower_S
        global upper_S
        upper_S = val
        upper_S = max(lower_S-1, upper_S)
        cv2.setTrackbarPos(upper_S_name,window_name,upper_S)
    cv2.createTrackbar(upper_S_name, window_name,upper_S,255,trackbar_upper_S)

    upper_V_name = 'V Upp'
    def trackbar_upper_V(val):
        global lower_V
        global upper_V
        upper_V = val
        upper_V = max(lower_V-1, upper_V)
        cv2.setTrackbarPos(upper_V_name,window_name,upper_V)
    cv2.createTrackbar(upper_V_name, window_name,upper_V,255,trackbar_upper_V)

def getCameraCoordinates():
    """Returns camera coordinates in meters, [x,y,depth]"""

    position = rs.rs2_deproject_pixel_to_point(intrinsics,[centroid_x,centroid_y],depth_image[centroid_y,centroid_x])
    
    for i, coord in enumerate(position):
        position[i] = position[i] * depth_scale
    
    return position

def isValidDepth(depth):
    return (depth > 0.15) and (depth < 0.5)

def getPenInRobotCoordinates():
    [Cx,Cy,Cd] = getCameraCoordinates()
    if not isValidDepth(Cd):
        return None
    Rx = ocx - Cx
    Ry = ocy - Cd
    Rz = ocz - Cy

    return [Rx,Ry,Rz]

def getWaistAngle(Rx,Ry):
    return np.arctan(Ry/Rx)

# Setup robot
robot = Robot()

ocx = 0.25689413363339614
ocy = 0.22700001078192145
ocz = 0.17914032450174594

following = False
relay = False
relayStart = False
lastFollowing = following


calFilename = "cal.csv"
hsvFilename = "hsv.csv"

try:
    with open(calFilename, 'r') as calFile:
        rdr = csv.reader(calFile)
        for row in rdr:
            ocx = float(row[0])
            ocy = float(row[1])
            ocz = float(row[2])
        print(f"Translation: ocx = {ocx}, ocy = {ocy}, ocz = {ocz}")
except Exception:
    print("Error reading calibration file")

try:
    with open(hsvFilename,'r') as hsvFile:
        rdr = csv.reader(hsvFile)
        for i, row in enumerate(rdr):
            if i == 0:
                lower_H = int(row[0])
                lower_S = int(row[1])
                lower_V = int(row[2])
            elif i == 1:
                upper_H = int(row[0])
                upper_S = int(row[1])
                upper_V = int(row[2])
except Exception:
    print("Error reading HSV file")

# Streaming loop
try:
    while True:

        # HSV values
        pen_lower = np.array([lower_H,lower_S,lower_V])
        pen_upper = np.array([upper_H,upper_S,upper_V])

        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        color_image_with_tracking = copy.deepcopy(color_image)

        # Threshold HSV image to get only purple
        mask = cv2.inRange(hsv_image,pen_lower,pen_upper)

        # Get contours
        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        validContour = True

        # Make sure we have contours
        if len(contours) <= 0:
            validContour = False

        else:
            # Get largest contours
            largest_contour = max(contours, key = cv2.contourArea)

            # calculate moments
            moments = cv2.moments(largest_contour)

            # Not valid if this moment is 0
            if moments['m00'] == 0:
                validContour = False
            else:  
                # Calculate centroids
                centroid_x = int(moments['m10']/moments['m00'])
                centroid_y = int(moments['m01']/moments['m00'])

                # Add contours to image
                color_image_with_tracking = cv2.drawContours(color_image_with_tracking, [largest_contour], 0, (0,255,0), 3)

                # Add centroid to color image
                color_image_with_tracking = cv2.circle(color_image_with_tracking, (centroid_x,centroid_y), radius=10, color=(0, 0, 255), thickness=-1)
        
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        bg_removed_tracking = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image_with_tracking)

        # Bitwise-AND mask and original image
        mask_result = cv2.bitwise_and(color_image,color_image,mask=mask)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #images = np.hstack((color_image,mask_result))
        #images = np.hstack((bg_removed,mask_result, depth_colormap))
        #images = np.hstack((bg_removed, depth_colormap))
        #images = np.hstack((mask,mask))
        #images = np.hstack((color_image,color_image_with_tracking))
        images = np.hstack((color_image_with_tracking,bg_removed_tracking, depth_colormap))

        #cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, images)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        elif key & 0xFF == ord('g'):
            robot.ctrl.gripper.grasp(delay=3)
        elif key & 0xFF == ord('o'):
            robot.ctrl.gripper.release(delay=3)
            
        elif key & 0xFF == ord('h'):
            robot.ctrl.arm.go_to_home_pose()
        elif key & 0xff == ord('s'):
            robot.ctrl.arm.go_to_sleep_pose()
        
        elif key & 0xFF == ord('c'):
            
            if validContour:
                [Cx,Cy,Cd] = getCameraCoordinates()
                print(f"Camera: Cx = {Cx}, Cy = {Cy}, Cd = {Cd}")
                [Rx,Ry,Rz] = robot.getEndEffector()
                print(f"Robot: Rx = {Rx}, Ry = {Ry}, Rz = {Rz}")
                ocx = Rx + Cx
                ocy = Ry + Cd
                ocz = Rz + Cy
                print(f"Translation: ocx = {ocx}, ocy = {ocy}, ocz = {ocz}")

                with open(calFilename,'w') as calFile:
                    wrtr = csv.writer(calFile)
                    wrtr.writerow([ocx,ocy,ocz])
        elif key & 0xFF == ord('='):
            with open(hsvFilename,'w') as hsvFile:
                wrtr = csv.writer(hsvFile)
                wrtr.writerow([lower_H,lower_S,lower_V])
                wrtr.writerow([upper_H,upper_S,upper_V])


        elif key & 0xFF == ord('f'):
            following = not following

        elif key & 0xFF == ord('r'):
            following = True
            relay = True
            
        elif key & 0xFF == ord('p'):
            [Cx,Cy,Cd] = getCameraCoordinates()
            print(f"Camera: Cx = {Cx}, Cy = {Cy}, Cd = {Cd}")
            [Rx,Ry,Rz] = robot.getEndEffector()
            print(f"Robot: Rx = {Rx}, Ry = {Ry}, Rz = {Rz}")
            robotPenCoordinates = getPenInRobotCoordinates()
            if robotPenCoordinates is not None:
                Px = robotPenCoordinates[0]
                Py = robotPenCoordinates[1]
                Pz = robotPenCoordinates[2]
                print(f"Pen: Px = {Px}, Py = {Py}, Pz = {Pz}")
        
        elif key & 0xFF == ord('t'):
            robot.ctrl.arm.go_to_home_pose()
            robot.absMove("elbow",-40)
            robot.absMove("waist",160)
            robot.absMove("elbow",10)

        elif key & 0xFF == ord('y'):
            robot.absMove("elbow",-40)
            robot.absMove("waist",0)
            robot.ctrl.arm.go_to_sleep_pose()


        if lastFollowing != following:
            
            if following:
                print("Start following")
                robot.ctrl.gripper.release(2)
                robot.ctrl.arm.go_to_sleep_pose()
                robot.relMove("shoulder", 10)
                robot.relMove("elbow", -10)
            else:
                print("Stop following")

            lastFollowing = following
                

        if following:
            robotPenCoordinates = getPenInRobotCoordinates()
            if robotPenCoordinates is not None:
                [Rx,Ry,Rz] = robot.getEndEffector()
                Px = robotPenCoordinates[0]
                Py = robotPenCoordinates[1]
                Pz = robotPenCoordinates[2]

                waistAngle = getWaistAngle(Px,Py)
                
                waistTolerance = np.deg2rad(2)
                xTolerance = 25/1000
                zTolerance = 50/1000


                print(f'Px = {Px}, Rx = {Rx}')
                if np.abs(robot.getJointCommand("waist") - waistAngle) > waistTolerance:
                    print("Adjust angle")
                    robot.ctrl.arm.set_single_joint_position('waist',waistAngle)
                elif (np.abs(Px - Rx) > xTolerance) or (np.abs(Pz - Rz) > zTolerance):

                    try:
                        print("Reach for pen")
                        robot.ctrl.arm.set_ee_cartesian_trajectory(x=Px-Rx,z=Pz-Rz)
                    except Warning:
                        robot.ctrl.arm.set_ee_cartesian_trajectory(x=10,z=10)
                elif (np.abs(Px - Rx) < xTolerance) and np.abs(robot.getJointCommand("waist") - waistAngle) < waistTolerance:
                    print("Grip")
                    robot.ctrl.gripper.grasp(2)
                    following = False
                    relayStart = relay

        if relayStart:
            time.sleep(10)
            robot.ctrl.arm.go_to_home_pose()
            robot.absMove("elbow",-40)
            robot.absMove("waist",160)
            robot.absMove("elbow",10)
            time.sleep(30)
            robot.ctrl.gripper.release(2)
            time.sleep(10)
            robot.absMove("elbow",-40)
            robot.absMove("waist",0)
            robot.ctrl.arm.go_to_sleep_pose()
            relayStart = False
            relay = False


finally:
    pipeline.stop()
    