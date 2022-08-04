# 3D Object Detection and Tracking
The software provides 3D object detection and tracking using [PAL MINI](https://dreamvu.com/pal-mini/). PAL MINI is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception.

## System Requirements
* Jetpack 4.6

## Installation

The Package can be installed in two ways:

### Method 1. Using Debian packages

The Package can be downloaded directly from [here](https://github.com/DreamVu/ppa/raw/main/3d-object-detection-tracking/3d-object-detection-tracking-pal-mini) and installed by running the below command from the location where it is downloaded,

    chmod +x 3d-object-detection-tracking-pal-mini && ./3d-object-detection-tracking-pal-mini

### Method 2. Using PPA Repository

The Package can be installed by adding the PPA Repository. Steps are as follows:

#### Step 1. Adding DreamVu PPAs
    sudo wget -qO - https://dreamvu.github.io/ppa/KEY.gpg | sudo apt-key add -
    sudo wget -qO /etc/apt/sources.list.d/dreamvu.list https://dreamvu.github.io/ppa/dreamvu.list
    
#### Step 2. Installing PAL MINI
    sudo apt update
    sudo apt install 3d-object-detection-tracking-pal-mini


Once complete please reboot the system. The packages will be installed in \~/DreamVu folder. 


## ROS Melodic Installations

The Package can be installed by running the below command after installing PAL MINI,

    sudo apt update
    sudo apt install dreamvu-melodic-tracking

## Documentation 
For rest of the evaluation of the 3D Object Detection and Tracking SDK, please read the 
- [Code Samples](https://docs.google.com/document/d/e/2PACX-1vR7AxhhOOp9K8PDviGaXRaw3Ui5E7omyL_hnvdsyWF_3dowyrgx8Zmc1mH1FOV3nsmt_HmEuBDpl-ZZ/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc) or you can email us directly at support@dreamvu.com 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
