# dddmr_web_nav

Web navigation panel for the DDDMR global planner plus EGO local planner setup.

## Run

```bash
ros2 launch dddmr_web_nav web_nav.launch.py
```

Then open:

```text
http://<robot-ip>:8080
```

## Default Topics

- `/mapground`: source traversable ground point cloud
- `/web/mapground`: downsampled Web marker published by this package
- `/drone_0_grid/grid_map/occupancy_inflate`: EGO local inflated obstacle point cloud
- `/web/ego_obstacles`: downsampled Web marker for EGO local obstacles
- `/baselink2map`: robot odometry
- `/global_path`: DDDMR global path
- `/clicked_point`: target point published by the Web panel
- `/web_estop`: Web emergency-stop signal
- `map`: default target frame for clicked points

The Web panel does not publish `/cmd_vel`. Motion still goes through the existing DDDMR + EGO + Go2 command chain.
