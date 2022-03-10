ROS Wrapper for PAL

- First download PAL SDK and run install script.

- Move dreamvu_pal_tracking package in the src directory of catkin workspace(/catkin_ws/src).

- Open a terminal and build the package:

        $ cd ~/catkin_ws
        $ catkin_make
        $ source ./devel/setup.bash
        
- To launch the tracking node along with an Rviz preview, open a terminal from catkin workspace and run:
        
        $ source ./devel/setup.bash
        $ roslaunch dreamvu_pal_tracking tracking_rviz.launch
   
 - To launch the following node along with an Rviz preview, open a terminal from catkin workspace and run:
        
        $ source ./devel/setup.bash
        $ roslaunch dreamvu_pal_tracking following_rviz.launch
 
 - To launch the object tracking node along with an Rviz preview, open a terminal from catkin workspace and run:
        
        $ source ./devel/setup.bash
        $ roslaunch dreamvu_pal_tracking tracking_objects_rviz.launch
 
 - To launch the object following node along with an Rviz preview, open a terminal from catkin workspace and run:
        
        $ source ./devel/setup.bash
        $ roslaunch dreamvu_pal_tracking following_objects__rviz.launch

- To launch the object detection node along with an Rviz preview, open a terminal from catkin workspace and run:
        
        $ source ./devel/setup.bash
        $ roslaunch dreamvu_pal_tracking detection_objects_rviz.launch              
