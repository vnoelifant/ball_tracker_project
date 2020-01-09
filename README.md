# ball_tracker_project
**MSR Hackathon project**


## Introduction
In this project, Python, ROS, and OpenCV were used to program a servo-controlled pan-tilt camera system that can track target objects. 

#### Objective
To track a blue ball with a camera mounted on 2 servos that are commanded to move such that the ball stays in the center of the camera frame.

#### Ball Tracking
The ROS camera driver used to get a live video feed is supported by the [usb_cam](http://wiki.ros.org/usb_cam) package. Once a feed is obtained, each image is converted to the HSV color space for better color segmentation using OpenCV. The centroid of the largest contour in each filtered image is determined and is passed along to the servo node.

#### Servo Control
With the centroid information passed to this node, the servos can be controlled such that the centroid is always in the center of the camera frame. The servos are mounted on a [pan-tilt mechanism](https://www.sparkfun.com/datasheets/Robotics/Other/sensor%20pan%20tilt%20manual.jpg) and are controlled through the [Pololu Micro Maestro servo controller](https://www.pololu.com/docs/0J40/1.a). 

## Implementation
#### Launch
[`launch.launch`](launches/launch.launch)

#### Nodes
##### Ball Tracking Node
[`camera_pak_node.py`](scripts/ball_tracker.py)

Subscribed Topic: `/camera_driver/image_raw`

Published Topic: `/center_position`

##### Servo Control Node
[`motor_control.py`](scripts/motor_control.py)

Subscribed Topic: `/center_position`

Click below to see a test in action. A blue ball is detected from the system. 

[![IMAGE ALT TEXT](http://img.youtube.com/vi/M0TucFrqqSo/0.jpg)](http://www.youtube.com/watch?v=M0TucFrqqSo "Object Tracking Project")

