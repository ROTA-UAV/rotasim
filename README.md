## rotasim
rotasim is a simulator with a focus on UAV testing and development.

# How to build
JSBSim:
```
cmake -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -mtune=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native -mtune=native" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=build/install -Bbuild -S.
cmake --build build --config Release
cmake --install build
```
MAVSDK
```
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=build/install -Bbuild -S.
cmake --build build --config Release
cmake --install build
```
