/*

CODE SAMPLE # 003: Object Tracking panorama
This code will grab the left panorama with object tracking data overlayed on it and would be displayed in a window using opencv


>>>>>> Compile this code using the following command....


g++ 003_object_tracking.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 003_object_tracking.out -I../include/ -w -std=c++11


>>>>>> Execute the binary file by typing the following command...


./003_object_tracking.out


>>>>>> KEYBOARD CONTROLS:

ESC key closes the window
       

*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>

#include <unistd.h>

using namespace cv;
using namespace std;
using namespace std::chrono;

# include <csignal>

bool init_done = false;

void signalHandler( int signum )
{
    printf("Interrupt signal %d\n", signum); 
    if(init_done) PAL::Destroy();
    exit(signum);
}

int main( int argc, char** argv )
{

    namedWindow( "PAL Object Tracking", WINDOW_NORMAL ); // Create a window for display.
    
    int width, height, camera_index = -1, model_id = 1;
	bool EnableDepth = true;
    PAL::Mode mode = PAL::Mode::OBJECT_TRACKING;

    if(PAL::Init(width, height, camera_index, EnableDepth, model_id, &mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }
    init_done = true;
    
    usleep(10);
    
    PAL::CameraProperties data; 
    PAL::Acknowledgement ack = PAL::LoadProperties("./SavedPalProperties.txt", &data);
	if(ack != PAL::SUCCESS)
	{
	    printf("Error Loading settings\n");
	}
	
    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at one fourth their original resolution.
    //Since the panoramas are vertically stacked, the window height should be twice of 1/4th height
    resizeWindow("PAL Object Tracking", width, height);
    
    int key = ' ';

    printf("\nPress ESC to close the window.\n");   

    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL Object Tracking", output);
    
    //27 = esc key. Run the loop until the ESC key is pressed

    while(key != 27)
    {
        PAL::Data::TrackingResults data;
        data = PAL::GrabTrackingData();

        cv::Mat output = data.left;
        //Display the concatenated image
        imshow( "PAL Object Tracking", output);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}

