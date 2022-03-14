/*

CODE SAMPLE # 005: Objject detection panorama
This code will grab the left panorama with object detection data overlayed on it and would be displayed in a window using opencv


>>>>>> Compile this code using the following command....


g++ 005_object_detection.cpp /usr/src/tensorrt/bin/common/logger.o  ../lib/libPAL.so  ../lib/libPAL_DE.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_Track.so `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 005_object_detection.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -I../../monolith/inc -w -std=c++11


>>>>>> Execute the binary file by typing the following command...


./005_object_detection.out


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


int main( int argc, char** argv )
{

    namedWindow( "PAL Object detection", WINDOW_NORMAL ); // Create a window for display.
    
    int width, height, camera_index = -1, model_id = 1;
	bool EnableDepth = false;
    PAL::Mode mode = PAL::Mode::OBJECT_DETECTION;

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
	
    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at one fourth their original resolution.
    //Since the panoramas are vertically stacked, the window height should be twice of 1/4th height
    resizeWindow("PAL Object detection", width, height);
    
    int key = ' ';
    
    printf("Press ESC to close the window.\n");   

    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL Object detection", output);
    
    //27 = esc key. Run the loop until the ESC key is pressed

    while(key != 27)
    {
        PAL::Data::TrackingResults data;
        data = PAL::GrabDetectionData();

        cv::Mat output = data.left;
        
        //Display the concatenated image
        imshow( "PAL Object detection", output);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}

