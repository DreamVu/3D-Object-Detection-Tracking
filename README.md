# 3D Object Detection and Tracking 
The software provides 3D object detection and tracking using [PAL Max](https://dreamvu.com/pal-ethernet/). PAL Max is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

To control PAL Max, users may follow one of the following methods:

## Method 1
Install the package on a host device. Using this method, users have access to our APIs and can run C++ code samples as well as sample ROS nodes.

### Supported Host Platforms 
Nvidia Jetson embedded boards with various [Jetpack SDK versions](https://developer.nvidia.com/embedded/jetpack-sdk-46)
Intel x86_64 system with Ubuntu 18.04/20.04 OS

Please follow the instructions given below on any of the supported devices to install the software on the host machine. 

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
Connect PAL Max properly to the host machine via the ethernet. Select the `DirectConnect` profile for wired connections. Option to select the profile can be found on the top-right corner under wired connections section.


### Documentation 
For rest of the evaluation of the 3D Object Detection and Tracking SDK, please read the 
- [Code Samples](https://docs.google.com/document/d/e/2PACX-1vR7AxhhOOp9K8PDviGaXRaw3Ui5E7omyL_hnvdsyWF_3dowyrgx8Zmc1mH1FOV3nsmt_HmEuBDpl-ZZ/pub)

## Method 2
Run the application directly on Pal Max and interface using the ROS multi-device functionality. Using this method requires no additional installations.

### Requirements
- Host System(s) with ROS installed. See [ros/installation](http://wiki.ros.org/ROS/Installation) for more details

### Steps to start ROS sample node
- Select the system to make the ros master and run `roscore` on the selected system
- Run the following command
```
ssh -tt dreamvu@192.168.0.175 "export ROS_MASTER_URI=<$MASTER_URI> ; bash  /home/dreamvu/3D-Object-Detection-Tracking/<execution_script> $model_id $enable_depth"
```
Refer the documentation for more details.

### Documentation 
For rest of the evaluation of the 3D Object Detection and Tracking SDK, please read the following document
- [ROS Application Note](https://docs.google.com/document/d/e/2PACX-1vSd1vVj0cE2x1AviwNHMXRtkABDGT5LO6sl-0vEMzUZpGoGi1QYRcIYhyzhZ4Q1YjJ23IucpzSyLNam/pub)


## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc).

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
