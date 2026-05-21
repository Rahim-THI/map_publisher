import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import json
import math
import os
import numpy as np
from filterpy.kalman import KalmanFilter

OBJECT_LIST_PATH = os.path.join(os.path.dirname(__file__), '20241126_0017_crossing1_00.json')
PLAYBACK_INTERVAL = 0.1
MAP_OFFSET_X = 88.0
MAP_OFFSET_Y = -234.0

OBJECT_COLORS = {
    "Car":        (0.0, 0.6, 1.0),
    "Van":        (0.0, 0.4, 0.8),
    "Truck":      (1.0, 0.2, 0.0),
    "Trailer":    (0.8, 0.1, 0.0),
    "Pedestrian": (1.0, 1.0, 0.0),
    "Cyclist":    (1.0, 0.5, 0.0),
    "EScooter":   (0.5, 1.0, 0.5),
    "Animal":     (0.8, 0.4, 0.0),
    "Other":      (0.7, 0.7, 0.7),
}

def yaw_to_quaternion(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return 0.0, 0.0, sy, cy

def create_kalman_filter(dt=0.1):
    """
    Kalman filter for 2D position smoothing.
    State: [x, y, vx, vy]
    Measurement: [x, y]
    """
    kf = KalmanFilter(dim_x=4, dim_z=2)

    # State transition matrix (constant velocity model)
    kf.F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1,  0],
        [0, 0, 0,  1]
    ])

    # Measurement matrix (we observe x, y only)
    kf.H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    # Measurement noise (how noisy is the sensor)
    kf.R = np.eye(2) * 0.5

    # Process noise (how much we trust the motion model)
    kf.Q = np.eye(4) * 0.1

    # Initial covariance
    kf.P = np.eye(4) * 1.0

    return kf

class ObjectPublisher(Node):
    def __init__(self):
        super().__init__('object_publisher')

        self.declare_parameter('map_offset_x',      MAP_OFFSET_X)
        self.declare_parameter('map_offset_y',      MAP_OFFSET_Y)
        self.declare_parameter('playback_interval', PLAYBACK_INTERVAL)
        self.declare_parameter('object_list_path',  OBJECT_LIST_PATH)

        self.offset_x = self.get_parameter('map_offset_x').value
        self.offset_y = self.get_parameter('map_offset_y').value
        interval      = self.get_parameter('playback_interval').value
        data_path     = self.get_parameter('object_list_path').value

        self.get_logger().info(f"Offset: ({self.offset_x}, {self.offset_y})")
        self.get_logger().info("Kalman filter enabled for position smoothing")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, '/object_markers', qos)

        with open(data_path, 'r') as f:
            raw = json.load(f)

        self.tracks = raw['tracks']
        self.get_logger().info(f"Loaded {len(self.tracks)} tracks")

        ts_set = set()
        for tr in self.tracks:
            ts_set.update(tr['timestamps'])
        self.timestamps = sorted(ts_set)
        self.get_logger().info(f"Timeline: {len(self.timestamps)} steps")

        from collections import Counter
        types = Counter(tr['object_type'] for tr in self.tracks)
        self.get_logger().info(f"Types: {dict(types)}")

        self.track_data = {}
        for tr in self.tracks:
            self.track_data[tr['track_id']] = {
                'type':         tr['object_type'],
                'all_dims':     tr['dimensions'],
                'base_dims':    tr['dimensions'][0],
                'ts_index':     {ts: i for i, ts in enumerate(tr['timestamps'])},
                'positions':    tr['positions'],
                'orientations': tr['orientations'],
                'kf':           None,  # Kalman filter (initialized on first use)
            }

        self.frame_idx = 0
        self.timer = self.create_timer(interval, self.publish_frame)
        self.get_logger().info("Object publisher ready → /object_markers")

    def get_smoothed_position(self, td, pos):
        """Apply Kalman filter to smooth position."""
        if td['kf'] is None:
            # Initialize filter with first position
            td['kf'] = create_kalman_filter(dt=PLAYBACK_INTERVAL)
            td['kf'].x = np.array([pos[0], pos[1], 0.0, 0.0])

        # Predict next state
        td['kf'].predict()

        # Update with measurement
        td['kf'].update(np.array([pos[0], pos[1]]))

        # Return smoothed position
        return float(td['kf'].x[0]), float(td['kf'].x[1])

    def publish_frame(self):
        if self.frame_idx >= len(self.timestamps):
            self.frame_idx = 0
            self.get_logger().info("Loop restart")
            # Reset all Kalman filters on loop
            for td in self.track_data.values():
                td['kf'] = None

        current_ts = self.timestamps[self.frame_idx]
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        active = 0
        lifetime_ns = int(0.5 * 1e9)

        for tid, td in self.track_data.items():
            if current_ts not in td['ts_index']:
                continue

            fi     = td['ts_index'][current_ts]
            pos    = td['positions'][fi]
            yaw    = td['orientations'][fi]
            dims   = td['all_dims'][fi] if fi < len(td['all_dims']) else td['base_dims']
            length, width, height = dims

            # Apply Kalman filter for smooth position
            sx, sy = self.get_smoothed_position(td, pos)

            rx = sx - self.offset_x
            ry = sy - self.offset_y
            rz = pos[2]

            obj_type = td['type']
            r, g, b  = OBJECT_COLORS.get(obj_type, (0.7, 0.7, 0.7))
            qx, qy, qz, qw = yaw_to_quaternion(yaw)

            # Bounding box
            box = Marker()
            box.header.frame_id    = "map"
            box.header.stamp       = now
            box.ns                 = "objects"
            box.id                 = tid
            box.type               = Marker.CUBE
            box.action             = Marker.ADD
            box.pose.position.x    = rx
            box.pose.position.y    = ry
            box.pose.position.z    = rz + height / 2.0
            box.pose.orientation.x = qx
            box.pose.orientation.y = qy
            box.pose.orientation.z = qz
            box.pose.orientation.w = qw
            box.scale.x            = length
            box.scale.y            = width
            box.scale.z            = height
            box.color.r            = r
            box.color.g            = g
            box.color.b            = b
            box.color.a            = 0.75
            box.lifetime.nanosec   = lifetime_ns
            marker_array.markers.append(box)

            # Text label
            label = Marker()
            label.header.frame_id  = "map"
            label.header.stamp     = now
            label.ns               = "labels"
            label.id               = tid + 10000
            label.type             = Marker.TEXT_VIEW_FACING
            label.action           = Marker.ADD
            label.pose.position.x  = rx
            label.pose.position.y  = ry
            label.pose.position.z  = rz + height + 0.5
            label.scale.z          = 0.6
            label.color.r          = 1.0
            label.color.g          = 1.0
            label.color.b          = 1.0
            label.color.a          = 1.0
            label.text             = f"{obj_type} #{tid}"
            label.lifetime.nanosec = lifetime_ns
            marker_array.markers.append(label)

            # Direction arrow (vehicles only)
            if obj_type in ("Car", "Van", "Truck", "Trailer"):
                arrow = Marker()
                arrow.header.frame_id    = "map"
                arrow.header.stamp       = now
                arrow.ns                 = "arrows"
                arrow.id                 = tid + 20000
                arrow.type               = Marker.ARROW
                arrow.action             = Marker.ADD
                arrow.pose.position.x    = rx
                arrow.pose.position.y    = ry
                arrow.pose.position.z    = rz + height / 2.0
                arrow.pose.orientation.x = qx
                arrow.pose.orientation.y = qy
                arrow.pose.orientation.z = qz
                arrow.pose.orientation.w = qw
                arrow.scale.x            = length * 0.9
                arrow.scale.y            = 0.25
                arrow.scale.z            = 0.25
                arrow.color.r            = 1.0
                arrow.color.g            = 1.0
                arrow.color.b            = 1.0
                arrow.color.a            = 0.9
                arrow.lifetime.nanosec   = lifetime_ns
                marker_array.markers.append(arrow)

            active += 1

        self.pub.publish(marker_array)
        self.get_logger().info(f"[{self.frame_idx:04d}] ts={current_ts:.1f} | active={active}")
        self.frame_idx += 1

def main():
    rclpy.init()
    node = ObjectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
