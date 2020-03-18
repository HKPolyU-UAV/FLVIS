#  FLVIS
## Feedback Loop Based Visual Inertial SLAM

### 1-Demo Video
TBD
### 2-Relevent Publication:
TBD
### 3-Support Hardware/Dataset:
Intel RealSense D435i Camera <br />
EuRoC MAV Dataset
### 4-Build The Project
We have tested in the following environment: <br />
Ubuntu 16.04 + ROS Kinetic <br />
Ubuntu 18.04 + ROS melodic <br />
4.1 Clone the repository to the catkin work space eg. /catkin_ws/src
````
git clone https://github.com/Ttoto/FLVIS.git
````
4.2 Compile and Install 3rd Part library
````
cd catkin_ws/src/FLVIS/3rdPartLib/
./install3rdPartLib.sh
````
4.3 Compile
````
cd ~/catkin_ws
catkin_make
````
### 5-Verification
5.1 D435i Camera
5.2 EuRoC MAV Dataset
Download the dataset(say MH_05_difficult) into the bag folder:
````
roscd flvis/bag/
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_05_difficult/MH_05_difficult.bag
````
Edit the corresponding bag name in flvis_euroc_mav.launch file:
````
<node pkg="rosbag" type="play" name="rosbag" args="$(find flvis)/bag/MH_05_difficult.bag"/>
````
run the following launch files:
````
roslaunch flvis rviz.launch
roslaunch flvis flvis_euroc_mav.launch
````

### Maintainer:
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />
Yajing Zou(Dept.LSGI,PolyU):rick.zou@connect.polyu.hk
