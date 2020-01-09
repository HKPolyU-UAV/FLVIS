# FLVIS

## Build
### Installing dependencies

### Clone from this repo
Clone the repository to the catkin work space eg. /catkin_ws/src
````
git clone https://github.com/Ttoto/VO.git
````
### Compile and Install 3rd Part library
#### sparse library
````
sudo apt-get install libsuitesparse
````
#### yaml-cpp(0.6.2)
````
cd yaml-cpp-0.6.2
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j4
sudo make install

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
#### DBow3
````
cd DBow3
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
