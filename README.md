# Smart Traffic Data Collection - Map Visualization

**Group project - Summer Semester 2026**  
Technische Hochschule Ingolstadt | AImotion Bavaria  
Supervisor: Markus Geisler

## Description
This ROS2 package visualizes traffic data from three urban intersections
in Ingolstadt (High-Definition Test field) using Lanelet2 maps and object lists.
Objects are visualized with Kalman filter smoothing, color coding by type,
and direction arrows.

## Requirements
- Ubuntu 22.04 / WSL2
- ROS2 Humble
- Python 3.10+

### Python dependencies
```bash
pip install lanelet2 filterpy numpy
```

## Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/Rahim-THI/map_publisher.git
cd ~/ros2_ws
colcon build --packages-select map_publisher
source install/setup.bash
```

## Dataset
Download JSON files from EFS cloud and place them:
- Crossing 1: `map_publisher/map_publisher/20241126_0017_crossing1_00.json`
- Crossing 2: `urbaning_labels/20241126_0001_crossing2_00.json`
- Crossing 3: `urbaning_labels/20241127_0010_crossing3_08.json`

Map file: `map_publisher/map_publisher/crossings_lanelet2map.osm`

## Usage

### Launch everything with one command
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch map_publisher all_crossings.launch.py
```

### RViz Topics
Add these MarkerArray topics in RViz:
- `/lanelet_map` - Lanelet2 HD map
- `/object_markers_c1` - Crossing 1 objects
- `/object_markers_c2` - Crossing 2 objects
- `/object_markers_c3` - Crossing 3 objects

## Map Offsets
| Crossing | offset_x | offset_y |
|---|---|---|
| Crossing 1 | 88.0 | -234.0 |
| Crossing 2 | -183.04 | -322.9 |
| Crossing 3 | 43.41 | 62.88 |

## Object Types & Colors
| Type | Color |
|---|---|
| Car | Blue |
| Van | Dark Blue |
| Truck | Red |
| Trailer | Dark Red |
| Pedestrian | Yellow |
| Cyclist | Orange |
| EScooter | Light Green |

## Package Structure
## Author
Abdurrahim Khanaliyev (abk8103@thi.de)  
Technische Hochschule Ingolstadt
