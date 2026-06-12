import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs import Point
from gui.gui.widgets.video_widget import CameraManager
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Bool

from rov_msgs.msg import Measurement
from rov_msgs.srv import CameraManage

NUM_POINTS_NEEDED = 2

class MeasurementCalculator(Node):
    def __init__(self) -> None:
        super().__init__('measurement_calculator')
        self.point_cloud_subscriber = None
        self.create_subscription(Bool, 'measurement_start', self.
                                measurement_start_callback, QoSPresetProfiles.DEFAULT.value)

        self.measurement_publisher = self.create_publisher(
            Measurement, 'measurement_calculation', QoSPresetProfiles.DEFAULT.value
        )

        self.point_cloud_manager = CameraManager('manage_luxonis', CameraManage.Request.POINT_CLOUD)

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        points = read_points(msg)

        pcd = o3d.geometry.PointCloud()
        # Retrieve the x, y, and z out of the pointcloud
        pcd.points = o3d.utility.Vector3dVector(points[:,:3].astype(np.float64))

        # Retrieve the colors out of the pointcloud
        colors = (points[:, 3:] / 255.0).astype(np.float64)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        visualization = o3d.visualization.VisualizerWithKeyCallback()
        visualization.create_window()

        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1000, origin=[0, 0, 0]
        )
        visualization.add_geometry(coordinate_frame)

        # Add the point cloud to the visualization
        visualization.add_geometry(pcd)

        # Pauses the thread until the window is closed
        visualization.run()

        # Destroys the window
        visualization.destroy_window()

        selected_points = visualization.get_picked_points()

        if selected_points.length == NUM_POINTS_NEEDED:
            # Retrieve the points from the list
            point1 = points[selected_points[0]]
            point2 = points[selected_points[1]]

            # Find the distance, divide by 10 so that it is in cm
            distance = np.linalg.norm(point1 - point2) / 10.0

            # Make the ros messages to send
            ros_point1 = Point(x=point1[0], y=point1[1], z=point1[2])
            ros_point2 = Point(x=point2[0], y=point2[1], z=point2[2])

            measurement = Measurement(point1=ros_point1, point2=ros_point2, distance=distance)

            # Publish the message
            self.measurement_publisher.publish(measurement)


            # Only turn off point clouds and destroy the subscriber if 2 points that way it is
            # possible to close the window to retrieve a different pointcloud

            # Turn off the point clouds
            self.point_cloud_manager.set_cam_state(on=False)

            # Destroy the point cloud subscription
            self.point_cloud_subscriber.destroy()
            self.point_cloud_subscriber = None

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

