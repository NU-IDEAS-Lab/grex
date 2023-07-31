#!/bin/bash

ros2 launch grex simulation.launch.py simulator_launch_file:=/home/gyaan/mas_simulation/src/grex/launch/simulator/flatland/simulator.launch.yaml agent_count:=4 simulator_agent_integration_launch_file:=/home/gyaan/mas_simulation/src/grex/launch/simulator/flatland/agent.launch.py use_rviz:=False 
