# VO


## Install
Clone the repository to the catkin work space eg. /catkin_ws/src
````
git clone https://github.com/Ttoto/VO.git
````
### install 3 part library
#### sparse library
````
sudo apt-get install libsuitesparse
````
#### g2o
````
cd g2o
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
#### Sophus
````
cd Sophus
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
### Compile
````
cd ~/catkin_ws
catkin_make
````
