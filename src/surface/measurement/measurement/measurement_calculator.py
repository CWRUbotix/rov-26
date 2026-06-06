import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Bool

from rov_msgs.msg import Measurement


class MeasurementCalculator(Node):
    def __init__(self) -> None:
        super().__init__('measurement_calculator')
        self.point_cloud_subscriber = None
        self.create_subscription(Bool, 'measurement_start', self.
                                measurement_start_callback, QoSPresetProfiles.DEFAULT.value)

        self.measurement_publisher = self.create_publisher(
            Measurement, 'measurement_calculation', QoSPresetProfiles.DEFAULT.value
        )

        # Thread(target=self.read_serial, daemon=True, name='Serial Reader').start()

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        points = read_points(msg)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))



    def measurement_start_callback(self) -> None:
        if self.point_cloud_subscriber is None:
            self.point_cloud_subscriber = self.create_subscription(
                PointCloud2, 'rgbd/point_cloud',
                self.point_cloud_callback, QoSPresetProfiles.DEFAULT.value
            )

def main() -> None:
    """Run the measurement calculator node."""
    rclpy.init()

    measurement_calculator = MeasurementCalculator()
    rclpy.spin(measurement_calculator)


if __name__ == '__main__':
    main()
