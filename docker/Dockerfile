FROM ros:jazzy-ros-base-noble as base

## Parameters
ENV WORKSPACE_ROOT=/capabilities2

ARG GITHUB_USERNAME
ARG GITHUB_PAT

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

WORKDIR /

RUN apt-get update -y
RUN apt-get install -y --no-install-recommends ros-dev-tools \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox

RUN apt-get clean

RUN git clone https://github.com/leethomason/tinyxml2.git

WORKDIR /tinyxml2
RUN make install


#############################################################################################################################
#####
#####   Clone packages and setup dependencies
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone -b capabilities2-server-fabric https://${GITHUB_USERNAME}:${GITHUB_PAT}@github.com/CollaborativeRoboticsLab/capabilities2.git
RUN git clone -b develop https://${GITHUB_USERNAME}:${GITHUB_PAT}@github.com/CollaborativeRoboticsLab/std_capabilities.git
RUN git clone https://${GITHUB_USERNAME}:${GITHUB_PAT}@github.com/CollaborativeRoboticsLab/nav_stack.git
RUN git clone -b ros2/devel https://${GITHUB_USERNAME}:${GITHUB_PAT}@github.com/CollaborativeRoboticsLab/prompt_tools.git
RUN git clone https://github.com/CollaborativeRoboticsLab/hri_audio_msgs.git

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list

RUN rosdep init && rosdep update && rosdep install --from-paths ${WORKSPACE_ROOT}/src -y --ignore-src


#############################################################################################################################
#####
#####   Build Capabilities packages
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}

RUN . /opt/ros/jazzy/setup.sh && colcon build

WORKDIR /

#############################################################################################################################
#####
#####   Remove workspace source and build files that are not relevent to running the system
#####
#############################################################################################################################

RUN rm -rf ${WORKSPACE_ROOT}/src
RUN rm -rf ${WORKSPACE_ROOT}/log
RUN rm -rf ${WORKSPACE_ROOT}/build

RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*
RUN apt-get clean


#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start final release image
#----
#---------------------------------------------------------------------------------------------------------------------------


FROM ros:jazzy-ros-base-noble as final

## Parameters
ENV WORKSPACE_ROOT=/capabilities2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY --from=base / /

COPY docker/workspace_entrypoint.sh /workspace_entrypoint.sh

RUN chmod +x /workspace_entrypoint.sh

ENTRYPOINT [ "/workspace_entrypoint.sh" ]

CMD ["/bin/bash"]