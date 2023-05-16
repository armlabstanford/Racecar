# Racecar
>Here is a set up guide for onboard development using Jetson Orin Nano. Should also work for most of other jetson boards, but no guarentees.

## [Setting up Git]
- Configure local client
    ~~~bash
    git config --global user.name "your_github_username"
    git config --global user.email "your_github_email"
    git config -l
    ~~~

- Then clone Racecar repo, put in your personal access token in password
    ~~~bash
    git clone https://github.com/armlabstanford/Racecar.git
    > Cloning into `YOUR-REPOSITORY`...
    Username: <type your username>
    Password: <type your password or personal access token (GitHub)
    ~~~
- Go to repo and save your credential
    ~~~bash
    git pull -v
    git config --global credential.helper cache
    ~~~

## [Setting up ROS Noetic]
- [Refer to this ROS official guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Also need ddynamic reconfig
    ~~~bash
    sudo apt install ros-noetic-ddynamic-reconfigure
    ~~~

## [Setting up mamba env]
- Install mamba
    ~~~bash
    cd ~/Downloads
    wget https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-pypy3-Linux-aarch64.sh .
    bash Mambaforge-pypy3-Linux-aarch64.sh
    ~~~
- Create environment and install packages
    ~~~bash
    mamba create -n racecar python=3.9
    mamba activate racecar
    pip install pythoncrc numpy evdev
    ~~~
- Don't forget to go to pyvesc to disable the get firmware line.

## [Setting up OpenCV]
- Download opencv-3.16 opencv-contrib-3.16 from github
- Create a folder named opencv/ in home directory, unzip both to folder
- Install necessary packages
    ~~~bash
    sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
          libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
          libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
          gfortran openexr libatlas-base-dev python3-dev python3-numpy \
          libtbb2 libtbb-dev libdc1394-22-dev
    ~~~
- Go to opencv/ execute commmands below:
    ~~~bash
    mkdir -p build && cd build
    cmake -DWITH_CUDA=ON -DENABLE_FAST_MATH=1 -DCUDA_FAST_MATH=1 -DWITH_CUBLAS=1 -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.4.16/modules ../opencv-3.4.16/
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local  -D INSTALL_C_EXAMPLES=ON  -D INSTALL_PYTHON_EXAMPLES=ON  -D OPENCV_GENERATE_PKGCONFIG=ON  -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.4.16/modules -D BUILD_EXAMPLES=ON ../opencv-3.4.16
    make -j6
    ~~~
- Now go to the cmakelist.txt of Racecar/ change opencv_dir to your build path. E.g:
    ~~~bash
    set(OpenCV_DIR /home/racecar/Documents/opencv/build)
    ~~~

## [Setting up librealsense]
- Intel has no official release for arm64 cpu, so we have to build it ourself
- We are referring to [this guide](https://www.lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/) to build the librealsense ourself.
    ~~~bash
    sudo apt-get update && sudo apt-get -y upgrade
    sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
    cd ~/Downloads
    git clone https://github.com/IntelRealSense/librealsense.git
    cd ./librealsense
    # Remove all realsense cameras before run
    ./scripts/setup_udev_rules.sh
    # Start build
    mkdir build && cd build
    cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
    sudo make uninstall && sudo make clean && sudo make -j6 && sudo make install
    ~~~