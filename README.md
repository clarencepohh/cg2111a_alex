# CG2111A Alex rescue robot

## 1. Arduino

## 2. RPi Communication with Arduino
```
gcc alex-pi.cpp serial.cpp serialize.cpp –pthread –o Alex-pi
./Alex-Pi
```

## 3. RPi RPLidar
```
source ~/cg2111a/devel/setup.bash (put in bashrc)
```

### 3.1.1 ROS networking (RPi)
```
roscore

roslaunch rplidar_ros rplidar.launch
```

### 3.1.2 ROS networking (desktop)
```
roslaunch hector_slam_launch tutorial.launch
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
