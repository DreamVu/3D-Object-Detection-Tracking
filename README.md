# 3D Object Detection and Tracking 
The software provides 3D object detection and tracking using [PAL Max](https://dreamvu.com/pal-ethernet/). PAL Max is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

### Supported Devices
- Nvidia Jetson embedded boards with various [Jetpacks](https://developer.nvidia.com/embedded/jetpack-sdk-46)
- Intel x86_64 system with Ubuntu 18.04 OS
- Intel x86_64 system with Ubuntu 20.04 OS

Please follow the instructions given below on any of the supported devices to install the software.

## Step 1. Clone the repository 
     sudo apt-get install git-lfs
     git clone -b PAL-Max --single-branch https://github.com/DreamVu/3D-Object-Detection-Tracking.git
     cd 3D-Object-Detection-Tracking
     git lfs pull
      

## Step 2. Installing PAL-Max SDK
      cd installations
      chmod +x ./*.sh
      ./install.sh
            
Once complete please reboot the system. 

## Steps to connect to the device
      Make sure that PAL Max camera is properly connected via the ethernet.
      Select the `DirectConnect` profile for wired connections. Option to select the profile can be found on the top-right corner under wired connections section.


## Documentation 
For rest of the evaluation of the 3D Object Detection and Tracking SDK, please read the 
- [Code Samples](https://docs.google.com/document/d/e/2PACX-1vR7AxhhOOp9K8PDviGaXRaw3Ui5E7omyL_hnvdsyWF_3dowyrgx8Zmc1mH1FOV3nsmt_HmEuBDpl-ZZ/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc).

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
