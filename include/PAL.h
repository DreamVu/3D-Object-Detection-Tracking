# ifndef PAL_H
# define PAL_H

//This is the only file the end-user needs to include in the application
//All the functionality provided by the API is covered here.

# include <vector>

# include "DataExchange.h"
# include "CameraProperties.h" 

namespace PAL
{

	//Initializes the PAL API
	//returns SUCCESS/FAILURE etc.
	PAL::Acknowledgement Init(int& panoramaWidth, int& panoramaHeight, int cameraIndex = -1, bool EnableDepth = true, int model_num = 1, void* arg=nullptr);


	//Writes the current camera properties into the provided memory location
	PAL::Acknowledgement GetCameraProperties(PAL::CameraProperties* properties);
	

	// SetCameraProperties
	// changes the camera properties like gamma, saturation etc.
	//
	// ARGUMENTS:
	// flags (read/write)		: Should point to a value formed by one/more combinations of CameraPropertyFlags
	// properties (readonly)	: Only those members are updated, who correspondings flags are set.
	//
	/* EXAMPLE:

		PAL::CameraProperties properties;
		properties.saturation = 2.0f;
		properties.gamma = 30000.0f;
		int flags = PAL::SATURATION | PAL::GAMMA;
		PAL_SDK::SetCameraProperties(&properties, &flags);
	*/
	// RETURNS:
	// 
	// returns SUCCESS/INVALID_PROPERTY_VALUE etc.
	// On successful return, flags should point to zero.
	// In case if an invalid properties are sent, 
	// the corresponding CameraPropertyFlags would be set to the int location pointed by flags
	// Refer API Doc for more information about this function
	PAL::Acknowledgement SetCameraProperties(PAL::CameraProperties* properties, unsigned int *flags);

	//Returns a vector of available resolutions. 
	//While changing the resolution through SetCameraProperties...
	//users can use one of the available resolutions
	std::vector<PAL::Resolution> GetAvailableResolutions();

	//Saves the current camera properties into the provided file name
	PAL::Acknowledgement SaveProperties(const char* fileName);
	
	//Loads the camera properties saved in the provided file
	//If data argument is provided, the properties in the file would be written into data
    PAL::Acknowledgement LoadProperties(const char* fileName, PAL::CameraProperties* data = 0);
	
	//Destroys all the resources related to Camera communication	
	void Destroy();
	void ExitDevice();

	//Grabs the left and right panorama data
	PAL::Data::Stereo GetStereoData();

	//Grabs the left, right panorama along with the tracking data. If enableDepth  \
	//is enbled then it will also grab the depth and floor panoramas.
	PAL::Data::TrackingResults GrabTrackingData();

	//Grabs the left, right panorama along with only the detection data. If enableDepth  \
	//is enbled then it will also grab the depth and floor panoramas.
	PAL::Data::TrackingResults GrabDetectionData();


	//Set the Tracked ID you want to follow. \
	//Need to be FOLLOWING or OBJECT_FOLLOWING mode. Else returns -2. 
	void SetTrackID(int id);

	//Get the Tracked ID that is being followed. \
	//Need to be FOLLOWING or OBJECT_FOLLOWING mode. Else returns -2. 
	int GetTrackID();


}

# endif //PAL_H
