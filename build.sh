# build Pangolin
cd third_party
cd Pangolin
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

# build geographiclib
cd ../../
cd geographiclib
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8