g++ 001_people_tracking.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 001_people_tracking.out -I../include/ -w -std=c++11

g++ 002_people_following.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 002_people_following.out -I../include/ -w -std=c++11

g++ 003_object_tracking.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 003_object_tracking.out -I../include/ -w -std=c++11

g++ 004_object_following.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 004_object_following.out -I../include/ -w -std=c++11

g++ 005_object_detection.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -O3  -o 005_object_detection.out -I../include/ -w -std=c++11
