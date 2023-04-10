# Multi-Agent System Simulation Environment

The goal of this environment is to allow for eventual simulation of complex agents in a multi-agent system (MAS). While agents are currently very basic, the use of ROS 2 will allow for plug-and-play addition of new agent capabilities.

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
Installation via Docker may be performed as follows for use on systems where packages may not be installed.

Follow the steps below:

 1) Install `rocker`:
    ```bash
    pip install rocker
    ```

 2) Build the Docker image:
    ```bash
    nvidia-docker build . -t ros:mas-sim

 3) Launch a container using `rocker`:
    ```bash
    rocker --net=host --nvidia --x11 --user --home ros:mas-sim
    ```

 4) Change to the `mas_simulation` folder:

    ```bash
    cd ~/mas_simulation/
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

 1) Change to the `mas_simulation` root directory:

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
    ros2 launch simulation_base mas_simulation.launch.py agent_count:=3
    ```

    * *Note: To view available launch arguments, add a* `-s` *flag to the end of the command.*

# Development Notes
Source gazebo before running:

   ```bash
   . /usr/share/gazebo/setup.sh
   ```

Convert maps using:
   ```bash
   python3 map2gazebo_offline.py --map_dir ~/dev/northwestern/aamas_environment/src/patrolling_sim/maps/cumberland/cumberland.pgm --export_dir ~/dev/northwestern/
   ```

RViz fix:
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 ros2 launch configuration test.launch.py use_rviz:=true
   ```

Using the Docker method with Cyclone DDS on ideas2 server will currently result in gzserver crash because of https://github.com/eclipse-cyclonedds/cyclonedds/issues/479.
