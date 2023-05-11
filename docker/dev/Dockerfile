FROM thecanadianroot/opencv-cuda:ubuntu20.04-cuda11.3.1-opencv4.5.2-rosnoetic


# Update, Upgrade, Install, and Cleanup
RUN apt-get update && \
        apt-get install -y \
        python3-catkin-tools \
        wget \
        unzip \
        git \
        ros-noetic-pybind11-catkin \
        ros-noetic-control-msgs \
        ros-noetic-ros-controllers \
        ros-noetic-industrial-robot-status-interface \
        ros-noetic-scaled-joint-trajectory-controller \
        ros-noetic-ur-description \
        ros-noetic-tf-conversions 

WORKDIR /code/catkin_ws

CMD /bin/bash

