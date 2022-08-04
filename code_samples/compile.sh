g++ 001_people_tracking.cpp `pkg-config --libs --cflags opencv python3 libusb-1.0 objdet` -O3  -o 001_people_tracking.out 

g++ 002_people_following.cpp `pkg-config --libs --cflags opencv python3 libusb-1.0 objdet` -O3  -o 002_people_following.out 

g++ 003_object_tracking.cpp `pkg-config --libs --cflags opencv python3 libusb-1.0 objdet` -O3  -o 003_object_tracking.out 

g++ 004_object_following.cpp `pkg-config --libs --cflags opencv python3 libusb-1.0 objdet` -O3  -o 004_object_following.out 

g++ 005_object_detection.cpp `pkg-config --libs --cflags opencv python3 libusb-1.0 objdet` -O3  -o 005_object_detection.out 

