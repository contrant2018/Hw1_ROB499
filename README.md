# Running the ROS 2 Sine Wave System

## Setup
1. Navigate to the workspace:
   cd ~/ros2_ws
2. Build the workspace:
   colcon build --packages-select hw2
3. Source the setup script:
   . install/setup.bash
4. run nessacary scripts
- **Sine Wave Publishers (oscope):**
  - **ros2 run hw2 oscope_osc**: Publishes a normal 1 Hz sine wave on the topic `Sinewave`.
  - **ros2 run hw2 oscope_slow**: Publishes a slow 0.5 Hz sine wave on the topic `slow_wave`.
  - **ros2 run hw2 oscope_fast**: Publishes a fast 2 Hz sine wave on the topic `fast_wave`.

- **Limiter Nodes (limiter):**
  - **ros2 run hw2 limiter_osc**: Subscribes to `Sinewave` and publishes the limited output on `limited_wave`.
  - **ros2 run hw2 limiter_slow**: Subscribes to `slow_wave` and publishes the limited output on `limited_slow_wave`.
  - **ros2 run hw2 limiter_fast**: Subscribes to `fast_wave` and publishes the limited output on `limited_fast_wave`.

