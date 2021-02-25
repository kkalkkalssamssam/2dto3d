# 2Dto3D

## 3D point clouding using 2D lidar with ROS



[![3d point clouding using 2d lidar - team600](http://img.youtube.com/vi/iZ8qzG8tDQY/0.jpg)](https://youtu.be/iZ8qzG8tDQY?t=0s) 

## Guide
* Installation
```
$ cd ${ROS workspace}/src
$ git clone https://github.com/kkalkkalssamssam/2dto3d.git
$ cd $(ROS workspace)
$ catkin_make
```
- Set permission of USB port

Arduino
```
$ sudo chmod a+rw /dev/ttyUSB0
```
Lidar
```
$ sudo chmod a+rw /dev/ttyUSB1
```

- Scan
```
$ roslaunch project_2dto3d scan.launch bag_filename:=() min_r:=() max_r:=() height:=()
```

- Change Finder
```
$ roslaunch project_2dto3d change_finder.launch bag_filename:=() min_r:=() max_r:=() height:=()
```

- Degree Finder
```
$ roslaunch project_2dto3d degree_finder.launch bag_filename:=()
```
  
- Object Cluster
```
$ roslaunch project_2dto3d cluster.launch bag_filename:=()
```
#### bag_filename must be same.
