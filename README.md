# Multi-Agent System Simulation Environment

The goal of this environment is to allow for eventual simulation of complex agents in a multi-agent system (MAS). While agents are currently very basic, the use of ROS 2 will allow for plug-and-play addition of new agent capabilities.

Please reach out to Anthony Goeckner \<anthony.goeckner@northwestern.edu\> with any questions.


# Manual Installation Instructions (Recommended)

Installation is simple, but prerequisites must first be installed:

 * ROS 2: Follow the official instructions [here](https://docs.ros.org/en/humble/Installation.html) to install **ROS 2 Humble** to your system. Use the **desktop-full** variant, which includes the Gazebo simulator.

 * Colcon: This is the ROS 2 build system, but doesn't seem to be installed automatically. Install using the official instructions, [here](https://colcon.readthedocs.io/en/released/user/installation.html).

 * *Optional for Windows*: Users may find it easier to install prerequisites and operate the simulator using the Windows Subsystem for Linux (WSL). Install WSL, and then use it to install the Ubuntu version of ROS 2.

Once prerequisites are installed, perform the following:

 1) Use Git to download the simulation environment repository.

    ```
    git clone --recurse https://github.com/NU-IDEAS-Lab/mas_simulation.git
    ```
    
 2) Change to the `mas_simulation` folder:

    ```
    cd ./mas_simulation/
    ```

 3) Source the correct ROS 2 installation.
   
    ```
    source /opt/ros/humble/setup.bash
    ```
    
 4) Build the simulation environment using Colcon, the ROS 2 build system.

    ```
    colcon build
    ```

 5) Follow instructions below to run the simulation.


# Operation Instructions
To operate the simulator, perform the following steps:

 1) Change to the `mas_simulation` root directory:

    ```
    cd ./mas_simulation/
    ```

 2) Source the installed packages, plus the Gazebo simulator.
   
    ```
    source ./install/setup.bash
    source /usr/share/gazebo/setup.sh
    ```

 3) Run the simulation:

    ```
    ros2 launch simulation_base mas_simulation.launch.py agent_count:=3
    ```

    * *Note: To view available launch arguments, add a* `-s` *flag to the end of the command.*