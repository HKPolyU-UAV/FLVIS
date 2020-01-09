#  FLVIS
## Feedback Loop Based Visual Initial SLAM

### 1-Demo Video
TBD
### 2-Relevent Publication:
TBD
### 3-Verify:
User can varify the performance using a Inter RealSense D435i Camera.

### 4-Build The Project
We have tested in the following environment:
Ubuntu 16.04 + ROS Kinetic
Ubuntu 18.04 + ROS melodic
4.1 Clone the repository to the catkin work space eg. /catkin_ws/src
````
git clone https://github.com/Ttoto/FLVIS.git
````
4.2 Compile and Install 3rd Part library <br />
4.2.1 sparse library
````
sudo apt-get install libsuitesparse
````
4.2.2 yaml-cpp(0.6.2)
````
cd yaml-cpp-0.6.2
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j4
sudo make install

````
4.2.3 g2o
````
cd g2o
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
4.2.4 Sophus
````
cd Sophus
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
4.2.5 DBow3
````
cd DBow3
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
4.3 Compile
````
cd ~/catkin_ws
catkin_make
````

### Maintainer:
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />
Yajing Zou(Dept.LSGI,PolyU):rick.zou@connect.polyu.hk
