/*

CODE SAMPLE # 001: People Tracking
This code will grab the left panorama with person tracking data overlayed on it and would be displayed in a window using opencv


>>>>>> Compile this code using the following command....


g++ 001_people_tracking.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 001_people_tracking.out -I../include/ -w -std=c++11


>>>>>> Execute the binary file by typing the following command...


./001_people_tracking.out


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
    signal(SIGINT, signalHandler);

    namedWindow( "PAL People Tracking", WINDOW_NORMAL ); // Create a window for display.
    
    int width, height, camera_index = -1, model_id = 0;
	bool EnableDepth = true;
    PAL::Mode mode = PAL::Mode::TRACKING;

    if(PAL::Init(width, height, camera_index, EnableDepth, model_id, &mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }
    
    usleep(10);
    
    PAL::CameraProperties data; 
    PAL::Acknowledgement ack = PAL::LoadProperties("./SavedPalProperties.txt", &data);
	if(ack != PAL::SUCCESS)
	{
	    printf("Error Loading settings\n");
	}
    init_done = true;
	
    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at one fourth their original resolution.
    //Since the panoramas are vertically stacked, the window height should be twice of 1/4th height
    resizeWindow("PAL People Tracking", width, height);
    
    int key = ' ';
   
    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL People Tracking", output);    
    
    printf("\nPress ESC to close the window.\n");
    //27 = esc key. Run the loop until the ESC key is pressed
    while(key != 27)
    {
        PAL::Data::TrackingResults data;

        data = PAL::GrabTrackingData();

        cv::Mat output = data.left;

        //Display the concatenated image
        imshow( "PAL People Tracking", output);  
        
        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}

