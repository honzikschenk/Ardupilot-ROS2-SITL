import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        self.declare_parameter('in_topic', '/scan')
        self.declare_parameter('out_topic', '/scan_fixed')
        self.declare_parameter('window', 20)
        self.declare_parameter('min_rate', 2.0)
        self.declare_parameter('max_rate', 50.0)

        self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value or '/scan'
        self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value or '/scan_fixed'
        self.window = int(self.get_parameter('window').value)
        self.min_rate = float(self.get_parameter('min_rate').value)
        self.max_rate = float(self.get_parameter('max_rate').value)

        self.sub = self.create_subscription(LaserScan, self.in_topic, self._cb, 10)
        self.pub = self.create_publisher(LaserScan, self.out_topic, 10)

        self.last_stamp_ros = None
        self.periods = []
        self.get_logger().info(f"ScanRestamper: {self.in_topic} -> {self.out_topic}")

    def _cb(self, msg: LaserScan):
        now_ros_time = self.get_clock().now()
        now_msg = now_ros_time.to_msg()
        if self.last_stamp_ros is not None:
            dt = (now_ros_time - self.last_stamp_ros).nanoseconds / 1e9
            if dt > 0:
                self.periods.append(dt)
                if len(self.periods) > self.window:
                    self.periods.pop(0)
        msg.header.stamp = now_msg
        self.last_stamp_ros = now_ros_time

        if self.periods:
            avg_dt = sum(self.periods) / len(self.periods)
            if 1e-6 < avg_dt and self.min_rate <= 1/avg_dt <= self.max_rate:
                msg.scan_time = avg_dt
                n = len(msg.ranges)
                if n > 1:
                    msg.time_increment = avg_dt / n
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  # pragma: no cover
    main()
