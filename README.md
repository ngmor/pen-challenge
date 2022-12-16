# Northwestern MSR Hackathon Pen Challenge

The goal of this project was to program a [Trossen PincherX-100 Robot Arm](https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/px100.html) to reach for and grab a pen detected by an [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/).

To do this I made use of the PyRealSense and OpenCV Python libraries to analyze the image from the D435i and the [Modern Robotics](https://github.com/NxRLab/ModernRobotics) library and the PincherX's Python API to plan and execute the motion towards the pen.

## Video

[Demo](https://user-images.githubusercontent.com/113186159/208075915-71848134-6255-4922-b52e-5b4e424fac0d.mp4)

## Methodology
The D435i provides both color and depth information. To find the pen's location, I could determine the pixels representing the pen in the image by thresholding the color image based on a range of HSV values which represented the pen's color. Then I could find a contour around these pixels and find the 2D coordinates of the centroid of that contour in the image. Finally, the depth data from the D435i gave the last coordinate needed to describe the pen's 3D coordinates in the camera's reference frame.

[Pen Tracking](https://user-images.githubusercontent.com/113186159/208076177-cfa8ac7b-56a0-4503-b4e1-2842f3c7d51f.webm)

To determine the location of the pen in the robot's reference frame, I added a calibration routine. In this routine, I held the pen up to the robot's gripper. I then used forward kinematics to determine the position of the robot's gripper in the robot's frame. I simplified the calibration by assuming that the camera was oriented horizontally and at a 90° angle to the robot arm. With that assumption, a comparison of the camera's pen coordinates and the robot's gripper position allowed me to determine a transformation between the camera and the robot reference frames.

Knowing this information, I could then determine the location of the pen in the robot's reference frame regardless of where I placed it. From there, a simple algorithm adjusted the PincherX 100's waist angle and executed a trajectory to move the gripper to the pen. Once it was within a tolerance, the gripper closed on the pen.

## Software Architecture
- [pen.py](pen.py) - the main script that performs computer vision processing and takes keyboard commands to initiate certain robot actions.
- [robot.py](robot.py) - a module with a simple class to encapsulate all necessary functionality for the robot.
- [robot_manual.py](robot_manual.py) - manual control script for the PincherX's Python API, for testing.
- [cal.csv](cal.csv) - saved calibration values to relate the camera frame to the robot frame.
- [hsv.csv](hsv.csv) - saved HSV ranges for pen color detection.
