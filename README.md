# comp3431_ass2

## How to run the line follower (Ellen's code):
### Terminal 1:
$ roslaunch turtlebot3_gazebo self_driving.launch

### Terminal 2:
$ cd catkin_ws/src/comp3431-ass2 <br/>
$ python lineFollower.py

## How to run the edge detection:

### Terminal 1:
$ roscore

### Terminal 2:
$ cs </br>
$ roslaunch comp3431-ass2 self_driving.launch

### Terminal 3:
$ cs </br>
$ export AUTO_IN_CALIB=action </br>
$ export GAZEBO_MODE=true </br>
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch

### Terminal 4:
$ cs </br> 
$ export AUTO_DT_CALIB=action </br> 
$ roslaunch comp3431-ass2 turtlebot3_autorace_detect_edge.launch

### Terminal 5: (To view the image being published)
$ rqt_image_view </br> 
*Change the topic to detect/image_lane/compressed*
