sudo rm -rf build
rm lcm_position_vr_go2
mkdir build
cd build
cmake ..
make -j 8
cd ..
