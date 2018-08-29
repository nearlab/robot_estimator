# cans_robot_controller

**Note: This does not currently work due to a lack of complementary IMU library/ros package. Please use the no_imu branch instead**

## Installing Eigen
>wget http://bitbucket.org/eigen/eigen/get/3.3.5.tar.gz  
>tar -xvf 3.3.5.tar.gz  
>mv eigen* eigen  
>sudo mv eigen /usr/share  

## Installing ROS
### Ubuntu 16.04
Follow directions on http://wiki.ros.org/kinetic/Installation  
### Ubuntu 18.04
Follow directions on http://wiki.ros.org/melodic/Installation  
### Otherwise
Figure it out with google  
***Arjun I need you to add stuff about catkin_tools*** 

## Installing vicon_bridge
>cd ~/catkin_ws/src  
>git clone https://github.com/nearlab/vicon_bridge.git 

## Running
>source ~/catkin_ws/devel/setup.bash
>roslaunch vicon_bridge vicon.launch
>roslaunch robot_estimator estimator.launch

Also includes messages!
