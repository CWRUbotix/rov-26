import sys
import time

import cv2
import depthai as dai
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

Matlike = NDArray[np.uint8]



try:
    import open3d as o3d
except ImportError:
    sys.exit(
        f"Critical dependency missing: Open3D. Please install it using the command: '{sys.executable} -m pip install open3d' and then rerun the script."
    )

class MeasurementCam(Node):
    def __init__(self) -> None:
        super().__init__('measurement_cam')

        self.create_subscription(
            Bool, 'retrieve_pointcloud', self.retrieve_pointcloud, QoSPresetProfiles.DEFAULT.value
        )

        self.cam_publisher = self.create_publisher(
            Image, 'lux_raw/image_raw', QoSPresetProfiles.SENSOR_DATA.value
        )

        self.create_subscription(
            Bool, 'measurement_pipeline', self.control_pipeline, QoSPresetProfiles.DEFAULT.value
        )

        print('measurement launched')

        self.rgbd_queue: dai.MessageQueue|None = None
        self.pipeline: dai.Pipeline|None = None
        self.left_cam_queue: dai.MessageQueue|None = None
        self.bridge = CvBridge()

    def create_pipeline(self) -> None:
        # Create pipeline

        self.pipeline = dai.Pipeline()
        fps = 30
        # Define sources and outputs
        left = self.pipeline.create(dai.node.Camera)
        right = self.pipeline.create(dai.node.Camera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
        # stereo.initialConfig.setConfidenceThreshold(200)
        # stereo.setLeftRightCheck(True)
        stereo.setSubpixelFractionalBits(3)
        rgbd = self.pipeline.create(dai.node.RGBD).build()
        align = None
        # o3dViewer = p.create(O3DNode)
        left.build(dai.CameraBoardSocket.CAM_A)
        right.build(dai.CameraBoardSocket.CAM_D)
        out = None

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setRectifyEdgeFillColor(0)
        stereo.enableDistortionCorrection(True)
        # stereo.setAlphaScaling(1)

        # Linking
        left_cam_output = left.requestFullResolutionOutput()
        left_cam_output.link(stereo.left)
        self.left_cam_queue = left_cam_output.createOutputQueue(maxSize=1, blocking=False)
        right.requestFullResolutionOutput().link(stereo.right)
        platform = self.pipeline.getDefaultDevice().getPlatform()
        if platform == dai.Platform.RVC4:
            out = left.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
            align = self.pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(align.input)
            out.link(align.inputAlignTo)
            align.outputAligned.link(rgbd.inDepth)
        else:
            out = left.requestFullResolutionOutput(dai.ImgFrame.Type.RGB888i, 30)
            stereo.depth.link(rgbd.inDepth)
            out.link(stereo.inputAlignTo)
        out.link(rgbd.inColor)

        self.rgbd_queue = rgbd.pcl.createOutputQueue(maxSize=1, blocking=False)
        # rgbd.pcl.link(o3dViewer.inputPCL)

        # Deploy pipeline to device
        while True:
            try:
                self.pipeline.start()
                self.pipeline.__enter__()
            except RuntimeError as e:  # noqa: F841 (unused variable e for optional logging below)
                self.get_logger().warning(
                    'Error uploading to Luxonis cam, retrying '
                    '(see cam_driver to enable more details)...'
                )
                # Uncomment to get more details about errors
                # These are usually just "the cam is disconnected", but can be other things
                # self.get_logger().warning(str(e))
                continue
            break

    def retrieve_pointcloud(self, _data: Bool) -> None:
        print('getting points')
        if self.rgbd_queue is not None:
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            # vis.register_key_action_callback(81, key_callback)
            pcd = o3d.geometry.PointCloud()

            try:
                print('before sleep')
                time.sleep(5)
                inPointCloud = self.rgbd_queue.tryGet()
                print('got inputPCL pointcloud')
                time.sleep(5)
                print('after 2nd sleep')
            except dai.MessageQueue.QueueException:
                print('errored')
                return # Pipeline closed
            if inPointCloud is not None:
                points, colors = inPointCloud.getPointsRGB()
                pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
                colors = (colors / 255.0).astype(np.float64)
                pcd.colors = o3d.utility.Vector3dVector(np.delete(colors, 3, 1))
                vis.add_geometry(pcd)
                vis.run()
                vis.destroy_window()
                print(vis.get_picked_points())
                selected_points = vis.get_picked_points()
                point_array = np.asarray(pcd.points)

                point1 = point_array[selected_points[0]]
                point2 = point_array[selected_points[1]]

                distance = np.linalg.norm(point1 - point2)
                print(distance)


            else:
                print('pointcloud none')
        else:
            print('RGBD queue is none, please start pipeline before retrieving pointcloud')

    def spin(self) -> None:
        if self.left_cam_queue is not None:
            video_frame = self.left_cam_queue.tryGet()

            # Discard None (failed to get frame)
            if video_frame is None:
                return

            # Type narrow to make mypy happy
            if not isinstance(video_frame, dai.ImgFrame):
                self.get_logger().warn('Dequeued something other than an image frame, skipping')
                return

            time_msg = self.get_clock().now().to_msg()

            if video_frame is not None:
                img_msg = self.get_image_msg(video_frame.getCvFrame(), time_msg)
                self.cam_publisher.publish(img_msg)

    def get_image_msg(self, image: Matlike, time: Time) -> Image:
        """Convert cv2 image to ROS2 Image with CvBridge.

        Parameters
        ----------
        image : Matlike
            The image to convert
        time : Time
            The timestamp for the ros message

        Returns
        -------
        Image
            The ROS2 image message
        """
        inverted_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_msg: Image = self.bridge.cv2_to_imgmsg(inverted_image)
        img_msg.encoding = 'rgb8'
        img_msg.header.stamp = time
        return img_msg

    def control_pipeline(self, data: Bool) -> None:
        print('\n\n\n\n\n\nRecieved pipeline control')
        if data.data:
            if self.pipeline is None:
                print('creating pipeline')
                self.create_pipeline()
            else:
                self.get_logger().error('Pipeline already created')
        elif self.pipeline is not None:
            self.pipeline.__exit__(None, None, None)
            self.pipeline = None
            self.left_cam_queue = None
            self.rgbd_queue = None
            print('killed pipeline')
        else:
            self.get_logger().warn('Pipeline is already killed')


    def shutdown(self) -> None:
        """Free the device and any other resources."""
        if self.pipeline:
            self.pipeline.__exit__(None, None, None)


def main() -> None:
    rclpy.init()
    driver_node = MeasurementCam()
    executor = MultiThreadedExecutor()

    try:
        while True:
            rclpy.spin_once(driver_node, executor=executor, timeout_sec=0)
            driver_node.spin()
    finally:
        driver_node.shutdown()
