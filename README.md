# av-trajectory-planner
Project for 16-748 implementing CILQR for real-time vehicle trajectory planning.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.


### Setup

This project was tested on Ubuntu 16.04. Follow the setup guide below for the Planner and Simulator.

#### Planner
```
sudo apt update
sudo apt install -y cmake \
    build-essential \
    python3-dev \
    pkg-config \
    libjsoncpp-dev \
    python3-pip

pip3 install numpy==1.11.0

git clone https://github.com/QuantStack/xtl.git
cd xtl
mkdir build
cd build
cmake ..
sudo make install
cd ../../

git clone https://github.com/QuantStack/xtensor.git
cd xtensor
mkdir build
cd build
cmake ..
sudo make install
cd ../../

git clone https://github.com/QuantStack/xtensor-blas.git
cd xtensor-blas
mkdir build
cd build
cmake ..
sudo make install
cd ../../

git clone https://github.com/QuantStack/xtensor-blas.git
cd xtensor-blas
mkdir build
cd build
cmake ..
sudo make install
cd ../../

git clone https://github.com/xianyi/OpenBLAS.git
cd OpenBLAS
mkdir build
cd build
cmake ..
sudo make install
cd ../../


```

##### Clone Repo

Clone the repository:
```
# Make sure to clone with --recursive
git clone --recursive https://github.com/joshuajbennett/av-trajectory-planner.git
cd av-trajectory-planner
```
If you didn't clone with the `--recursive` flag then manuall clone the pybind submodule from the top-level directory:
```
git submodule update --init --recursive
```

##### Build
```
mkdir build
cd build
cmake ..
make
```

#### Simulator
```
sudo apt install -y ffmpeg \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    libportmidi-dev \
    libswscale-dev \
    libavformat-dev \
    libavcodec-dev \
    zlib1g-dev \
    libmtdev-dev

sudo apt install -y \
    libgstreamer1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good

pip3 install -I Cython==0.28.2
pip3 install matplotlib==3.0.2
pip3 install numpy==1.11.0
pip3 install scipy
pip3 install kivy

```

* Kivy 1.0.9
* Numpy 1.11.0
* Matplotlib 3.0.2
* Python 3.x

You can also install Kivy by following the instructions on this website https://kivy.org/doc/stable/installation/installation-linux.html


### Solver

```
Give examples
```
