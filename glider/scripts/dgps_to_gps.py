import rclpy
from rclpy.node import Node

from gps_msgs.msg import GPSFix
from dgps_msgs.msg import DifferentialNavSatFix
from sensor_msgs.msg import NavSatStatus


def differential_nav_sat_fix_to_gps_fix(msg: DifferentialNavSatFix) -> GPSFix:
    """Convert a DifferentialNavSatFix message to a GPSFix message."""
    fix = GPSFix()

    # --- Header ---
    fix.header = msg.nmea.header

    # --- Position (lat / lon / alt) ---
    fix.latitude  = msg.nmea.latitude
    fix.longitude = msg.nmea.longitude
    fix.altitude  = msg.nmea.altitude

    # --- Position covariance ---
    # NavSatFix carries a 3x3 ENU position covariance (row-major).
    # GPSFix uses the same layout, so copy it directly.
    fix.position_covariance      = list(msg.nmea.position_covariance)
    fix.position_covariance_type = msg.nmea.position_covariance_type

    # --- Heading (track) and its error ---
    # DifferentialNavSatFix provides a heading (degrees) and scalar covariance.
    # GPSFix stores course-over-ground as `track` and its 1-sigma error as
    # `err_track` (also degrees).
    fix.track     = float(msg.heading)
    fix.err_track = float(msg.heading_covariance) ** 0.5  # variance -> std-dev

    return fix


class DgpsConverterNode(Node):

    INPUT_TOPIC  = '/dgps/dfix'
    OUTPUT_TOPIC = '/dgps/fix'
    QUEUE_DEPTH  = 10

    def __init__(self):
        super().__init__('dgps_converter')

        self._pub = self.create_publisher(
            GPSFix,
            self.OUTPUT_TOPIC,
            self.QUEUE_DEPTH,
        )

        self._sub = self.create_subscription(
            DifferentialNavSatFix,
            self.INPUT_TOPIC,
            self._callback,
            self.QUEUE_DEPTH,
        )

        self.get_logger().info(
            f'dgps_converter ready: '
            f'{self.INPUT_TOPIC} (DifferentialNavSatFix) -> '
            f'{self.OUTPUT_TOPIC} (GPSFix)'
        )

    def _callback(self, msg: DifferentialNavSatFix) -> None:
        self._pub.publish(differential_nav_sat_fix_to_gps_fix(msg))


def main(args=None):
    rclpy.init(args=args)
    node = DgpsConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
