Obstacle Avoidance
==================
Project to allow a UAV to avoid obstacles by using Computer Vision.

Image processing is done on an Odroid U3 with OpenCV. Sensor communication is done via ROS and Mavlink

Development in progress...

How To Run
==================

Open a terminal and run ```roscore```

Open a second terminal and gain root access by running ```sudo -s```  (camera may require root access)                   
then ```roslaunch obstacle_avoidance uvccamera.launch```

In a third terminal run our rosnode via ```rosrun obstacle_avoidance obstacle_avoidance_node```
If this command fails to find our node you can navigate to the directory```catkin_ws/devel/lib/obstacle_avoidance``` and run the node manually by ```./obstacle_avoidance_node```

The ```obstacle_avoidance_node``` should now be running and performing image processing via a usb webcam
