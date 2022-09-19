#include <ros/ros.h>
#include <sstream> 
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <fcntl.h>
#include <unistd.h>

#include "dreamvu_pal_tracking/Properties.h"
#include "dreamvu_pal_tracking/Point.h"
#include "dreamvu_pal_tracking/Loc3D.h"
#include "dreamvu_pal_tracking/TrackND.h"
#include "dreamvu_pal_tracking/vec_TrackND.h"
#include "dreamvu_pal_tracking/vec_vec_TrackND.h"

using namespace std;
using namespace cv;


void print_tracking_info(const dreamvu_pal_tracking::vec_vec_TrackND& info)
{
    for(int i = 0; i < info.TrackND_V2.size(); i++)
    {
        if(i == 0)
            std::cout << "In OK" << std::endl;
        else if(i == 1)
            std::cout << "In Searching" << std::endl;
        else
            std::cout << "In Terminated" << std::endl;
        
        for(int j = 0; j < info.TrackND_V2[i].TrackND_V1.size(); j++)
        {
            std::cout << "t_is_activated: " << info.TrackND_V2[i].TrackND_V1[j].t_is_activated << std::endl;
            std::cout << "t_track_id: " << info.TrackND_V2[i].TrackND_V1[j].t_track_id << std::endl;
            std::cout << "active: " << info.TrackND_V2[i].TrackND_V1[j].active << std::endl;
            std::cout << "t_score: " << info.TrackND_V2[i].TrackND_V1[j].t_score << std::endl;
            std::cout << "t_label: " << info.TrackND_V2[i].TrackND_V1[j].t_label << std::endl;
            std::cout << "boxes.x1: " << info.TrackND_V2[i].TrackND_V1[j].boxes.x1 << std::endl;
            std::cout << "boxes.y1: " << info.TrackND_V2[i].TrackND_V1[j].boxes.y1 << std::endl;
            std::cout << "boxes.x2: " << info.TrackND_V2[i].TrackND_V1[j].boxes.x2 << std::endl;
            std::cout << "boxes.y2: " << info.TrackND_V2[i].TrackND_V1[j].boxes.y2 << std::endl;
            std::cout << "locations_3d.x: " << info.TrackND_V2[i].TrackND_V1[j].locations_3d.x << std::endl;
            std::cout << "locations_3d.y: " << info.TrackND_V2[i].TrackND_V1[j].locations_3d.y << std::endl;
            std::cout << "locations_3d.z: " << info.TrackND_V2[i].TrackND_V1[j].locations_3d.z << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;
}


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "sample_tracking_info_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber info_sub = nh.subscribe("/dreamvu/pal/tracking/get/info", 1, print_tracking_info);
    
    ros::spin();

    return 0;
}