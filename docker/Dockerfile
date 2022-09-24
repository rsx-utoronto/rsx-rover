# Based on: https://github.com/utiasASRL/Crystal_Ball_Nav/tree/master/Docker/noetic_pytorch_image
FROM nvcr.io/nvidia/cudagl:11.0-devel-ubuntu20.04

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
# RUN apt-get install wget
# RUN apt-key del A4B469963BF863CC
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
# RUN dpkg -i cuda-keyring_1.0-1_all.deb

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Setup environment
RUN apt-get update
RUN apt-get upgrade -y --no-install-recommends
RUN DEBIAN_FRONTEND=noninteractive

# Install preliminary packages
RUN apt-get update
RUN apt-get upgrade -y --no-install-recommends
RUN apt-get install gcc g++ python3 python3-pip python3-dev vtk7 -y --no-install-recommends
RUN apt-get install gedit firefox wget git xvfb vim nano -y --no-install-recommends
RUN apt-get update
RUN apt-get upgrade -y --no-install-recommends

# Install preliminary python packages
RUN pip3 install --upgrade pip
RUN pip3 install setuptools
RUN pip3 install numpy scikit-learn matplotlib PyYAML
RUN pip3 install munch

# Install pypcd
RUN pip3 install --upgrade git+https://github.com/klintan/pypcd.git
RUN pip3 install imageio

# Install PyTorch
RUN pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

# Additionnal stuff
RUN apt install ffmpeg -y --no-install-recommends
RUN pip3 install imageio imageio-ffmpeg

# Install ROS 
RUN apt-get update
RUN apt-get install curl gnupg2 lsb-release -y --no-install-recommends
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt update
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update

RUN apt-get update
RUN apt-get install ros-noetic-desktop-full -y --no-install-recommends

# Install ROS packages
RUN apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y --no-install-recommends
RUN rosdep init
RUN rosdep update

RUN apt-get install python3-catkin-tools -y --no-install-recommends
RUN apt-get install ros-noetic-velodyne* -y --no-install-recommends
RUN apt-get install ros-noetic-pointcloud-to-laserscan -y --no-install-recommends
RUN apt-get install ros-noetic-rgbd-launch -y --no-install-recommends
# RUN apt-get install ros-noetic-range-sensor-layer -y --no-install-recommends
RUN apt-get install ros-noetic-openni2* -y --no-install-recommends
RUN apt-get install ros-noetic-octomap* -y --no-install-recommends
RUN apt-get install ros-noetic-ros-control -y --no-install-recommends
RUN apt-get install ros-noetic-tf2* -y --no-install-recommends
RUN apt-get install ros-noetic-people-msgs -y --no-install-recommends
RUN apt-get install ros-noetic-people-velocity-tracker -y --no-install-recommends
RUN apt-get install ros-noetic-navigation* -y --no-install-recommends
# RUN apt-get install ros-noetic-social-navigation-layers -y --no-install-recommends
RUN apt-get install ros-noetic-teleop-twist-keyboard -y --no-install-recommends

# Additionnal packages
RUN apt-get install -y gnome-core mesa-utils --no-install-recommends

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN apt-get update
RUN apt-get upgrade -y --no-install-recommends

# Additionnal packages
RUN apt-get install psmisc -y --no-install-recommends
RUN apt-get install ros-noetic-ros-numpy -y --no-install-recommends
RUN pip3 install plyfile scipy jupyterlab

# Install gazebo
RUN apt-get install gazebo11
RUN apt-get install libgazebo11-dev -y --no-install-recommends
RUN apt-get install ros-noetic-gazebo* -y --no-install-recommends

RUN apt-get install python3-catkin-tools python3-osrf-pycommon -y --no-install-recommends

RUN apt-get update
RUN apt-get install ros-noetic-robot-localization -y --no-install-recommends
RUN apt-get install ros-noetic-twist-mux -y --no-install-recommends
RUN apt-get install ros-noetic-hector-gazebo-plugins -y --no-install-recommends
RUN apt-get install ros-noetic-python-qt-binding -y --no-install-recommends
RUN apt-get install ros-noetic-qt-gui-core -y --no-install-recommends
RUN apt-get install ros-noetic-rqt-graph -y --no-install-recommends
RUN apt-get install ros-noetic-costmap-converter -y --no-install-recommends

RUN pip3 install pynvml

# RUN apt-get install libnvidia-gl-430:i386

# Install Mayavi from source
RUN pip install mayavi
RUN pip install PyQt5

# Install mayavi and QT
RUN apt-get install pyqt5-dev-tools pyqt5-dev -y --no-install-recommends

RUN pip3 install open3d
RUN pip3 install autopep8 flake8

RUN export QT_PLUGIN_PATH=/usr/lib/qt/plugins
RUN echo "export QT_PLUGIN_PATH=/usr/lib/qt/plugins" >> ~/.bashrc

# Install opencv 

WORKDIR /workspace/compiled_libs
RUN apt-get install -y libopencv-dev python3-opencv qt5-default libvtk6-dev zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev \
                        libopenexr-dev libgdal-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev \
                        libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm \
                        libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev \
                        libtbb-dev 

RUN apt install -y wget unzip
# RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.1.zip
# RUN unzip opencv.zip && rm opencv.zip
# RUN mv opencv-4.5.1 opencv
# WORKDIR /workspace/compiled_libs/opencv
# RUN mkdir -p build
# WORKDIR /workspace/compiled_libs/opencv/build
# RUN cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON \
#       -DWITH_XINE=ON -DENABLE_PRECOMPILED_HEADERS=OFF ..
# RUN make -j4
# RUN make install
# RUN ldconfig

# Setup user information
ARG username0=bag
ARG userid0=1085
RUN useradd -ms /bin/bash -u $userid0 $username0
RUN adduser $username0 root
RUN adduser $username0 sudo
RUN echo $username0':rsx123' | chpasswd
USER $username0
RUN mkdir -p /home/$username0/rover-ws/src/rsx-rover

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "export USER=$username0" >> ~/.bashrc

# Some QT-Apps/Gazebo don't show controls without this
ENV QT_X11_NO_MITSHM 1

ENV ROS_DISTRO noetic
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
ENV USER=$username0


