# map_publisher

ROS2 package for visualizing HD map (Lanelet2) and traffic objects from the UrbanIng dataset in RViz2.

## Dependencies
- ROS2 Jazzy
- lanelet2
- visualization_msgs

## Build
```bash
cd ~/ros2_ws
colcon build --packages-select map_publisher
source install/setup.bash
```

## Run

**Terminal 1 - HD Map:**
```bash
ros2 run map_publisher lanelet_visualizer
```

**Terminal 2 - Traffic Objects (crossing1):**
```bash
ros2 run map_publisher object_publisher \
  --ros-args \
  -p map_offset_x:=88.0 \
  -p map_offset_y:=-234.0 \
  -p object_list_path:=/path/to/20241126_0017_crossing1_00.json
```

**Terminal 3 - RViz2:**
```bash
rviz2
```

## RViz2 setup
1. Fixed Frame → map
2. Add → /lanelet_map (MarkerArray)
3. Add → /object_markers (MarkerArray)

## Calibrated offsets
| Crossing  | map_offset_x | map_offset_y |
|-----------|-------------|-------------|
| crossing1 | 88.0        | -234.0      |

## Object colors
| Type       | Color       |
|------------|-------------|
| Car        | Blue        |
| Truck      | Red         |
| Pedestrian | Yellow      |
| Cyclist    | Orange      |
| Van        | Dark Blue   |
| EScooter   | Light Green |
