sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

distribution=$(lsb_release -sc)
if [[ $distribution == "bionic" ]]; then
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	ros_name=melodic

else
	sudo apt install curl # if you haven't already installed curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	ros_name=noetic
fi	

sudo apt update

if [[ $distribution == "bionic" ]]; then
	sudo apt install ros-melodic-desktop
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
else
	sudo apt install ros-noetic-desktop
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi

source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update

if [[ $distribution == "bionic" ]]; then
	source /opt/ros/melodic/setup.bash
else
	source /opt/ros/noetic/setup.bash
fi

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
