cat ../dreamvu_pal_tracking/cmake_template/header.txt > ../dreamvu_pal_tracking/CMakeLists.txt
echo "set(PAL_INCLUDE_DIR" `pwd`/../include ")" >> ../dreamvu_pal_tracking/CMakeLists.txt
echo "set(PAL_LIBRARY" `pwd`/../lib/libPAL.so ")" >> ../dreamvu_pal_tracking/CMakeLists.txt
distribution=$(lsb_release -sc)
if [[ $distribution == "bionic" ]]; then
	cat ../dreamvu_pal_tracking/cmake_template/footer.txt >> ../dreamvu_pal_tracking/CMakeLists.txt
else
	cat ../dreamvu_pal_tracking/cmake_template/footer2.txt >> ../dreamvu_pal_tracking/CMakeLists.txt
fi

