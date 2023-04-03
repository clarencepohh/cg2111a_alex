# CG2111A Alex rescue robot

## 1. Arduino

## 2. RPi Communication with Arduino
```
gcc alex-pi.cpp serial.cpp serialize.cpp –pthread –o Alex-pi
./Alex-Pi
```

## 3. RPi RPLidar
```
source ~/cg2111a/devel/setup.bash
```

### 3.1.1 ROS networking (RPi)
```
roscore
export ROS_MASTER_URI=http://192.168.1.1:11311

roslaunch rplidar_ros rplidar.launch
```

### 3.1.2 ROS networking (desktop)
```
source/opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.1.1:11311
rostopic list
rosrun rviz rviz
```

### 3.2 rviz on RPi
```
roslaunch rplidar_ros view_slam.launch
```

## 4 Debugging
### Add RPi to host
```
vim /etc/hosts
```
