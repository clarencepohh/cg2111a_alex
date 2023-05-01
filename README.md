# CG2111A Alex rescue robot
This README documents the main functions and debugging steps taken during EPP2 (CG2111A) Project.

## 1. Arduino

## 2. RPi Communication with Arduino
```
Compiling Alex RPi code on the RPi terminal: 
gcc alex-pi.cpp serial.cpp serialize.cpp –pthread –o Alex-pi

To run the Alex RPi code:
./Alex-Pi
```

## 3. RPi RPLidar
```
Initial source code to run for every new terminal opened. By putting in .bashrc we remove the need to do this every time:
source ~/cg2111a/devel/setup.bash (put in bashrc)
```

### 3.1.1 ROS networking (RPi)
```
Run roscore on RPi (one terminal):
roscore

Start RPLidar Data stream (another terminal):
roslaunch rplidar_ros rplidar.launch
```

### 3.1.2 ROS networking (desktop - Linux)
```
Run rviz on Ubuntu terminal:
roslaunch hector_slam_launch tutorial.launch

**note: IMPORT_MASTER_URI done in .bashrc**
```

### 3.2 rviz on RPi
```
To launch rviz on the RPi (no offloading to another desktop; we did not do this):
roslaunch rplidar_ros view_slam.launch
```

## 4 Debugging
### Add RPi to host
```
vim /etc/hosts
```
