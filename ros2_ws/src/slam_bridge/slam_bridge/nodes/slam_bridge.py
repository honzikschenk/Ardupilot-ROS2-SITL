import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from mavros_msgs.msg import ObstacleDistance


INPUT_TOPIC = '/mavros/obstacle_distance'
OUTPUT_TOPIC = '/scan'


class ObstacleDistanceToScan(Node):
    def __init__(self):
        super().__init__('slam_bridge')
        self._sub = self.create_subscription(ObstacleDistance, INPUT_TOPIC, self._cb, 10)
        self._pub = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)
        self.get_logger().info(f"Bridging {INPUT_TOPIC} -> {OUTPUT_TOPIC} (LaserScan)")

    def _cb(self, msg: ObstacleDistance):
        distances = getattr(msg, 'distances', [])
        if not distances:
            self.get_logger().warn('Empty obstacle distance array')
            return

        ranges = []
        for d in distances:
            if d in (0, 0xFFFF):
                ranges.append(float('inf'))
            else:
                ranges.append(d / 100.0)

        count = len(ranges)
        if count < 2:
            self.get_logger().warn('Too few readings for a scan')
            return

        inc_deg = float(getattr(msg, 'increment', 0.0) or getattr(msg, 'angle_increment', 0.0) or 0.0)
        offset_deg = float(getattr(msg, 'angle_offset', 0.0) or getattr(msg, 'angle_min', 0.0) or 0.0)

        angle_increment = math.radians(inc_deg) if inc_deg else (2 * math.pi / count)
        angle_min = math.radians(offset_deg)
        angle_max = angle_min + angle_increment * (count - 1)

        scan = LaserScan()
        scan.header = getattr(msg, 'header', Header())
        if not scan.header.frame_id:
            scan.header.frame_id = 'sf45b_lidar'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0

        min_d_cm = int(getattr(msg, 'min_distance', 5))
        max_d_cm = int(getattr(msg, 'max_distance', 5000))
        scan.range_min = min_d_cm / 100.0
        scan.range_max = max_d_cm / 100.0
        scan.ranges = ranges
        scan.intensities = []

        self._pub.publish(scan)
        self.get_logger().debug(
            f'Published LaserScan beams={count} inc={scan.angle_increment:.4f} rad range[{scan.range_min:.2f},{scan.range_max:.2f}]')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistanceToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()