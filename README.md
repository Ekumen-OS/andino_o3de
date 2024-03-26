# andino_o3de

<img src=docs/andino_warehouse.png width=500 />

## :clipboard: Description

[`Andino`](https://github.com/Ekumen-OS/andino) is a fully open-source diff drive robot designed for educational purposes and low-cost applications. It is fully integrated with ROS 2 and it is a great base platform to improve skills over the robotics field.

`andino_o3de` leverages the power of [O3DE](https://o3de.org/) for an andino simulation and
serves as an entry point for people looking for starting with O3DE Simulator.

## :clamp: Platforms

- OS:
  - Ubuntu 22.04 Jammy Jellyfish
- O3DE:
  - Tested on 23.10.2
- ROS 2: Humble Hawksbill

## :inbox_tray: Workspace setup

Refer to [workspace setup](WORKSPACE_SETUP.md) for setting up the workspace.

## :rocket: Usage

Open the O3DE Editor of the project. Refer to [workspace setup](WORKSPACE_SETUP.md).


### RViz Visualization

```
ros2 run rviz2 rviz2 -d andino_o3de_ws/rviz.rviz --ros-args -p use_sim_time:=true
```

<img src=docs/rviz.png width=500 />


### Teleoperate the robot

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### SLAM via andino_slam package


Once the simulation is up we can use the `andino` stack. Let's use `andino_slam` package to create a map.

1. Install `andino_slam` package.
    ```
    sudo apt install ros-humble-andino-slam
    ```

2. Run slam
    ```
    ros2 launch andino_slam slam_toolbox_online_async.launch.py
    ```

3. Visualize the creation of the map in RViz using the `map` display.

    <img src=docs/slam.png width=500 />
