# Grex Multi-Agent Environment

This environment allows for simulation of complex agents in a multi-agent system (MAS). We use ROS 2 to allow for plug-and-play addition of new agent capabilities.

<img width="600" alt="IDEAS MAS Framework Diagram" src="https://user-images.githubusercontent.com/1892393/232659638-5e71e73a-f2ef-44a0-92a9-b227886df2b7.png">

**Our goal is to make the following interchangeable:**

 * **Simulator:** Easily change between simulators, such as Gazebo or Stage. Use the same agents and environments in different simulators.
 * **Agents:** Define custom agents, from simple standalone ROS 2 nodes to complex agents involving tens of nodes and the myriad third-party packages offered by ROS.
 * **Networking:** We plan to integrate the NS-3 network simulator for realistic communication models and disturbances.


Please reach out to Anthony Goeckner \<anthony.goeckner@northwestern.edu\> with any questions.


# Manual Installation Instructions (Recommended)

Installation is simple, but prerequisites must first be installed:

 * ROS 2: Follow the official instructions [here](https://docs.ros.org/en/humble/Installation.html) to install **ROS 2 Humble** to your system. Use the **desktop-full** variant, which includes the Gazebo simulator.

   * Install the `Eclipse Cyclone DDS` middleware for ROS. We use this because the default middleware currently causes [problems](https://github.com/ros2/ros2/issues/1253) in our application.
     ```bash
     apt install ros-humble-rmw-cyclonedds-cpp
     ```
     The above will install Cyclone DDS for Ubuntu. Adapt as needed for your system.

   * Install other ROS 2 packages.
     ```bash
     apt install ros-dev-tools ros-humble-turtlebot3* ros-humble-navigation2*
     ```
     Adapt as needed for your system.

 * *Optional for Windows*: Users may find it easier to install prerequisites and operate the simulator using the Windows Subsystem for Linux (WSL). Install WSL, and then use it to install the Ubuntu version of ROS 2.
  
Once prerequisites are installed, perform the following:

 1) Use Git to download the simulation environment repository.

    ```bash
    git clone --recurse https://github.com/NU-IDEAS-Lab/mas_simulation.git
    ```
    
 2) Change to the `mas_simulation` folder:

    ```bash
    cd ./mas_simulation/
    ```

 3) Source the correct ROS 2 installation.
   
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    
 4) Build the simulation environment using Colcon, the ROS 2 build system.

    ```bash
    colcon build
    ```

 5) Follow instructions below to run the simulation.


# Docker Installation Instructions (Alternative Method)
Installation via Docker may be performed as follows for use on systems where packages cannot be installed.

Follow the steps below:

 1) Install `rocker`:
    ```bash
    pip install rocker
    ```

 2) Build the Docker image:
    ```bash
    docker build . -t grex:sim

 3) Launch a container using `rocker`:
    ```bash
    rocker --net=host --nvidia --x11 --user --home grex:sim
    ```

 4) Change to the `grex` folder:

    ```bash
    cd ~/grex/
    ```

 5) Source the correct ROS 2 installation.
   
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    
 6) Build the simulation environment using Colcon, the ROS 2 build system.

    ```bash
    colcon build
    ```


# Operation Instructions
To operate the simulator, perform the following steps:

 1) Change to the `grex` root directory:

    ```
    cd ./mas_simulation/
    ```

 2) Source the installed packages, plus the Gazebo simulator, and set environment variables.
   
    ```
    source ./install/setup.bash
    source /usr/share/gazebo/setup.sh
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

 3) Run the simulation:

    ```
    ros2 launch grex simulation.launch.py agent_count:=3
    ```

    * *Note: To view available launch arguments, add a* `-s` *flag to the end of the command.*
