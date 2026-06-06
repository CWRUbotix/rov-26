import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from rov_msgs.msg import Measurement


class MeasurementCalculator(Node):
    def __init__(self) -> None:
        super().__init__('measurement_calculator')
        self.create_subscription(
            PointCloud2, 'rgbd/point_cloud',
            self.point_cloud_callback, QoSPresetProfiles.DEFAULT.value
        )

        self.measurement_publisher = self.create_publisher(
            Measurement, 'measurement_calculation', QoSPresetProfiles.DEFAULT.value
        )

        # Thread(target=self.read_serial, daemon=True, name='Serial Reader').start()

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        self.serial.write(msg.command.encode())

    def ros_publish(self, packet: str) -> None:
        """Publish a message from the transceiver."""
        self.serial_publisher.publish(FloatSerial(serial=packet))

        if packet[: len(ROS_PACKET)] != ROS_PACKET:
            return

        try:
            if SerialReaderPacketHandler.is_ros_single_message(packet):
                single_msg = self.serial_packet_handler.handle_ros_single(packet)
                if single_msg:
                    self.ros_single_publisher.publish(single_msg)
            else:
                msg = self.serial_packet_handler.message_parser(packet)
                self.data_publisher.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Error {e} caught dropping packet')

def main() -> None:
    """Run the measurement calculator node."""
    rclpy.init()

    measurement_calculator = MeasurementCalculator()
    rclpy.spin(measurement_calculator)


if __name__ == '__main__':
    main()
