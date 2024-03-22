# andino_o3de

<img src=docs/andino_warehouse.png width=500 />

## Workspace setup

Refer to [workspace setup](WORKSPACE_SETUP.md) for setting up the workspace.

## Usage

Open the O3DE Editor of the project.


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
