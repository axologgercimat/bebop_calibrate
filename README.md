#   Demos: Simple camera calibration utility Bebop2

By:     Edgar I. Chávez-Aparicio
email:  edgar.chavez@cimat.mx

This is a simple library for camera calibration for the Parrot Bebop2
and ROS1-Noetic. Used by the CIMAT Robotics and Intelligent Systems group.

#   DEPENDENCIES

+ ROS1 - noetic     <http://wiki.ros.org/noetic/Installation/Ubuntu>
+ bebop_authonomy   <http://wiki.ros.org/bebop_autonomy>


##  Install ROS - noetic

```bash
sudo sh -c 'echo "deb http://archive.ubuntu.com/ubuntu focal universe" >> /etc/apt/sources.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox ros-noetic-mavros python3-rosdep python3-pip
pip3 install future
sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash

#   CREATE CATKIN - ROS WORKSPACE
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin init  # initialize your catkin workspace
wstool init

#   IF YOU WISH TO INITIALIZE CATKIN WORKSPACE AT BEGINING OF SESSION
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


##  Install bebop_autonomy

```bash
cd ~/catkin_ws/src
git clone https://github.com/AutonomyLab/parrot_arsdk.git src/parrot_arsdk
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
rosdep update
rosdep install --from-paths src -i
source /opt/ros/noetic/setup.bash

cd ~/catkin_ws
#   Leer nota 1)
catkin build
```

Nota 1)
tal vez sea necesario aplicar los siguientes parches en caso de error a parrot_arsdk:
+ arsdk_patch_1.patch
+ arsdk_patch_2.patch

Cómo aplicarlos:
```bash
cd ~/catkin_ws/src
git apply bebop_calibrate/patches/arsdk_patch_*.patch --directory=parrot_arsdk/
```

Puede que sea necesario corre`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/catkin_ws/src/devel/lib/parrot_arsdk` cada que se vuelva a cargar `source/devel/setup.bash`
o bien se puede dejar como configuración:

```bash
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$HOME/catkin_ws/src/devel/lib/parrot_arsdk" >> ~/.bashrc
source ~/.bashrc
```

#   INSTALLATION
```bash
cd ~/catkin_ws/src
git clone git@github.com:cimat-ris/bebop_calibrate.git
cd ~/catkin_ws
catkin build bebop_calibrate
```

#   TESTING

Para terminar cualquier programa: `ctrl + c`

```bash
# @ bebop_ws
# Screen 1
roslaunch bebop_calibrate bebop_calibrate_nodelet.launch ip:=192.168.45.1
# Screen 2
rosrun rqt_image_view rqt_image_view

#   Una vez terminado, correr:
python3 src/bebop_calibrate/scripts/calibrate.py src/bebop_calibrate/output

```
