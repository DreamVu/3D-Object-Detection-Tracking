#include <ros/ros.h>
#include <sstream> 
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <fcntl.h>
#include <unistd.h>

#include "std_msgs/Int32MultiArray.h"
#include "dreamvu_pal_tracking/Properties.h"
#include "CameraProperties.h"

using namespace std;
using namespace cv;

#define ALL_PROPERTIES 1


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "sample_properties_publisher");

    ros::NodeHandle nh;

    int width, height;

#if ALL_PROPERTIES
    ros::Publisher properties_pub = nh.advertise<dreamvu_pal_tracking::Properties>("/dreamvu/pal/set/properties_all", 1,0);
#else
    ros::Publisher properties_pub = nh.advertise<std_msgs::Int32MultiArray>("/dreamvu/pal/set/properties", 1,0);
#endif

    ros::Rate loop_rate(30);

    while(properties_pub.getNumSubscribers()==0)
    {
        ROS_WARN("Waiting for subscibers");
        sleep(1);
    }

#if ALL_PROPERTIES
    dreamvu_pal_tracking::Properties ros_prop;

    //Set the desired values to below properties 
    ros_prop.brightness = -10;
    ros_prop.contrast = 10;
    ros_prop.saturation =  40;
    ros_prop.gamma =  250;
    ros_prop.gain =  10;
    ros_prop.white_bal_temp = 6000;
    ros_prop.sharpness =  20; 
    ros_prop.exposure =  10; 
    ros_prop.auto_white_bal =  false; 
    ros_prop.auto_exposure =  true; // means that the auto exposure will be off
    ros_prop.resolution_width =  1120;
    ros_prop.resolution_height =  384;
    ros_prop.color_space =  0;
    ros_prop.power_line_frequency =  0;
    ros_prop.vertical_flip =  true;
    ros_prop.filter_disparity =  true; 
    ros_prop.filter_spots =  false; 
    ros_prop.fov_start =  0; 
    ros_prop.fov_end =  360; 
    ros_prop.projection =  true;
    ros_prop.computation =  1;
    ros_prop.camera_height =  200;
    ros_prop.fd =  true;

    //Set the flags to publish and apply selective properties to PAL. 
    //In this case all the properties will be changed.
    ros_prop.flags = PAL::ALL;
#else
    std_msgs::Int32MultiArray ros_prop;
    ros_prop.data.clear();

    ros_prop.data.push_back(1);
    ros_prop.data.push_back(14);
#endif

    properties_pub.publish(ros_prop);

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}