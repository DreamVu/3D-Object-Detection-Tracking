# 3D Object Detection and Tracking 
The software provides 3D object detection and tracking using [PAL Max](https://dreamvu.com/pal-ethernet/). PAL Max is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

*PAL-Max is an embedded camera. To control the camera, you may follow one of the following methods*
- Install the package on a host device and directly control the unit using socket connection over the ethernet.
- Run the application directly on the Pal-Max unit and interface using the ROS multi-device functionality.

## - Method 1
Installing the host package

### Supported Host Platforms 
- Nvidia Jetson embedded boards with various [Jetpack SDK versions](https://developer.nvidia.com/embedded/jetpack-sdk-46)
- Intel x86_64 system with Ubuntu 18.04 OS
- Intel x86_64 system with Ubuntu 20.04 OS

Please follow the instructions given below on any of the supporthttp://wiki.ros.org/ROS/Installationed devices to install the software on the host machine. 

### Step 1. Clone the repository 
     sudo apt-get install git-lfs
     git clone -b PAL-Max --single-branch https://github.com/DreamVu/3D-Object-Detection-Tracking.git
     cd 3D-Object-Detection-Tracking
     git lfs pull
      

### Step 2. Installing PAL-Max SDK
      cd installations
      chmod +x ./*.sh
      ./install.sh
            
Once complete please reboot the system. 

### Getting Started 
Connect the PAL Max camera properly to the host machine via the ethernet. Select the `DirectConnect` profile for wired connections. Option to select the profile can be found on the top-right corner under wired connections section.


### Documentation 
For rest of the evaluation of the 3D Object Detection and Tracking SDK, please read the 
- [Code Samples](https://docs.google.com/document/d/e/2PACX-1vR7AxhhOOp9K8PDviGaXRaw3Ui5E7omyL_hnvdsyWF_3dowyrgx8Zmc1mH1FOV3nsmt_HmEuBDpl-ZZ/pub)

## - Method 2
Using ROS to interface with no additional installations required

### Requirements
- Host System(s) with ROS installed. See [ros/installation](http://wiki.ros.org/ROS/Installation) for more details

### Steps to start ROS sample node
- Make sure the PAL-Max unit are directly connected or on the same local network
- Select the system to make the ros master and run `roscore` on the selected system
- Run the following command
```
ssh -tt dreamvu@192.168.0.175 "export ROS_MASTER_URI=<$MASTER_URI> ; bash  /home/dreamvu/3D-Object-Detection-Tracking/<execution_script> $model_id $enable_depth"
```
Refer the documentation for more details.

You may use [installations/setup_connection.sh](https://github.com/DreamVu/3D-Object-Detection-Tracking/blob/PAL-Max/installations/setup_connection.sh) to setup faster direct connection profile for the PAL-Max unit. After installing select the `DirectConnect` profile for wired connections.

### Documentation 
For rest of the evaluation of the 3D Object Detection and Tracking SDK, please read the 
- [ROS Samples](https://docs.google.com/document/d/e/2PACX-1vSd1vVj0cE2x1AviwNHMXRtkABDGT5LO6sl-0vEMzUZpGoGi1QYRcIYhyzhZ4Q1YjJ23IucpzSyLNam/pub)


## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc).

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
