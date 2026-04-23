FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-navigation2* \
    ros-jazzy-turtlebot3* \
    ros-dev-tools

