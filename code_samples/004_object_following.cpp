/*

CODE SAMPLE # 004: Object following panorama
This code sample will center on the object being tracked and follow him


>>>>>> Compile this code using the following command....


g++ 004_object_following.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 004_object_following.out -I../include/ -w -std=c++11


>>>>>> Execute the binary file by typing the following command...


./004_object_following.out


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

# include <csignal>

bool g_mExit = false;

void signalHandler( int signum )
{
    printf("Interrupt signal %d\n", signum); 
    printf("Closing. This may take a minute\n");
    g_mExit = true;
}

string getCmdOutput(string cmd)
{
   string outputString;
   FILE *outpStream;
   const int MaxLineLen = 128;
   char  outpLine[MaxLineLen];
   outpStream = popen(cmd.c_str(), "r");
   if (outpStream) {
      while (!feof(outpStream)) {
          if (fgets(outpLine, MaxLineLen, outpStream) != NULL) {
             outputString += outpLine;
          }
      }
      pclose(outpStream);
   }
   return outputString;
}

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

int main( int argc, char** argv )
{
    signal(SIGINT, signalHandler);

    namedWindow( "PAL Object Following", WINDOW_NORMAL ); // Create a window for display.
    
    int width, height, camera_index = -1, model_id = 1;
	bool EnableDepth = true;
    PAL::Mode mode = PAL::Mode::OBJECT_FOLLOWING;

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
    resizeWindow("PAL Object Following", width, height);
    
    int key = ' ';
    
    printf("\nPress ESC to close the window.\n");
    printf("Press i/I to enter the id you want to follow.\n");
    printf("Press p/P to print the id that is being followed.\n");

    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL Object Following", output);
    
    //27 = esc key. Run the loop until the ESC key is pressed

    while(key != 27 && !g_mExit)
    {
        PAL::Data::TrackingResults data;

        data = PAL::GrabTrackingData();

        cv::Mat output = data.left;

        //Display the concatenated image
        imshow( "PAL Object Following", output);  
        
        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;

        if (key == 'i' || key == 'I')
        {
            string zenityCmd = "zenity --entry --text \"Enter ID to Track\" --title \"PAL Object Following\" --entry-text=\"\"";
            string output = getCmdOutput(zenityCmd);

            output.erase(std::remove(output.begin(), output.end(), ' '), output.end());
            output.erase(std::remove(output.begin(), output.end(), '\n'), output.end());
            output.erase(std::remove(output.begin(), output.end(), '\t'), output.end());

            if(is_number(output))
            {
                PAL::SetTrackID(stoi(output));
            }    
            else
            {
                string errorCmd = "zenity --warning  --text \"Not a valid input\" --title \"PAL Object Following\"";
                FILE *outpStream = popen(errorCmd.c_str(), "r");
                pclose(outpStream);      
            }
        }
        if (key == 'p' || key == 'P')
        {
            int new_id = PAL::GetTrackID();
            if(new_id == -2)
            {
                std::cout << "Currently not in Following Mode" << std::endl;
            }
            else if(new_id == -1)
            {
                std::cout << "No ID has been entered. Press 'i' to enter an ID" << std::endl; 
            }
            else
            {
                std::cout << "The ID currently being followed is: " << new_id << std::endl;
            }
        }
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}

