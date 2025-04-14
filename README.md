# Running the ROS 2 Sine Wave System

## Setup
1. Navigate to the workspace:
   cd ~/ros2_ws
2. Build the workspace:
   colcon build --packages-select hw2
3. Source the setup script:
   . install/setup.bash

## Running the Nodes
1. Start the sine wave node:
   ros2 run hw2 oscope
2. Start the limiter node:
   ros2 run hw2 limiter
