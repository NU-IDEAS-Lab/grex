FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-navigation2* \
    ros-humble-turtlebot3* \
    ros-dev-tools

