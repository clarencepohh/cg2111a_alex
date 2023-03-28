# CG2111A Alex rescue robot

## 1. Arduino

## 2. RPi Communication with Arduino
```
gcc Alex-pi.cpp serial.cpp serialize.cpp –pthread –o Alex-pi

./Alex-Pi
```

## 3. RPi RPLidar
```
source ~/cg2111a/devel/setup.bash

roslaunch rplidar_ros view_slam.launch
```