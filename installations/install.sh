chmod +x ./*.sh

./dependencies.sh

./setup_connection.sh

./ros_cmake.sh

if [[ $(uname -p) == "x86_64" ]]; then 
	cp ../lib/libfiles/libPAL_x86.so ../lib/libPAL.so
else 
	cp ../lib/libfiles/libPAL_arm.so ../lib/libPAL.so
fi


