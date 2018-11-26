# av-trajectory-planner
Project for 16-748 implementing CILQR for real-time vehicle trajectory planning.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.



### Prerequisites

This project was tested on Ubuntu 16.04 and has the following prerequisite

#### Planner
```
sudo apt update
sudo apt install cmake \
                 build-essential \
                 python3-dev \
                 pkg-config \
                 libjsoncpp-dev \
                 python3-pip y
             
pip3 install numpy
                 
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

* Kivy 1.0.9
* Numpy 1.11.0
* Matplotlib 3.0.2
* Pythoon 3.x

You can install Kivy by following the instructions on this website https://kivy.org/doc/stable/installation/installation-linux.html

```
pip install matplotlib==3.0.2
pip install numpy==1.11.0
```

#### Solver

```
Give examples
```
