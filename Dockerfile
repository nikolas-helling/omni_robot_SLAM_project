# pull docker image for the base installation of ROS melodic
FROM ros:melodic-ros-base

# create catkin workspace and set working dir
WORKDIR /omni_robot_slam_ws
RUN mkdir -p src

# install system dependencies and ROS dependencies
RUN apt-get update && apt-get install -y \
    # ros dependencies
    ros-melodic-desktop-full \
    ros-melodic-gmapping \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-plotjuggler \
    ros-melodic-plotjuggler-ros \
    # system dependencies
    apt-utils \
    build-essential \
    terminator \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    dbus-x11 \
    x11-apps \
    x11-xserver-utils \
    # python dependencies
    python-pip \
    python-rosdep \
    python-dev \
    python-pyqt5 \
    python-tk \
    python-catkin-tools \
    libjpeg-dev \
    zlib1g-dev && \
    # initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && \
    rosdep update && \
    # install ROS dependencies with rosdep
    rosdep install --from-paths src --ignore-src -r -y && \
    # clean up
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# copy ROS packages into the workspace
COPY ./omni_robot_odom /omni_robot_slam_ws/src/omni_robot_odom
COPY ./slam_gmapping_amcl /omni_robot_slam_ws/src/slam_gmapping_amcl
COPY ./ira_laser_tools /omni_robot_slam_ws/src/ira_laser_tools
COPY ./data /omni_robot_slam_ws/src/data

# automate sourcing for new shells and build catkin ws
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /omni_robot_slam_ws/devel/setup.bash" >> ~/.bashrc && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# default command
CMD ["/bin/bash"]