cp -r ~/DreamVu/3D-Object-Detection-Tracking-PAL-USB/dreamvu_pal_tracking/ ~/catkin_ws/src/
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
roslaunch dreamvu_pal_tracking following_objects_rviz.launch
