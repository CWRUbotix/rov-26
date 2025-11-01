import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

# --- constants ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 400

LEFT_CAM_SOCKET = 0  # Camera A
RIGHT_CAM_SOCKET = 1  # Camera D


class FramePublishers:
    """Manage publishing frames to ROS2 topics."""

    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        self.publishers = {
            'left': self.node.create_publisher(Image, 'lux_left/image_raw', 10),
            'right': self.node.create_publisher(Image, 'lux_right/image_raw', 10),
        }

    def publish(self, topic: str, frame: dai.ImgFrame):
        if frame is None:
            return
        img_msg = self.bridge.cv2_to_imgmsg(frame.getCvFrame(), encoding='rgb8')
        img_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publishers[topic].publish(img_msg)


class LuxonisCamDriverNode(Node):
    def __init__(self):
        super().__init__('luxonis_cam_driver')
        self.frame_publishers = FramePublishers(self)

        # Detect connected devices
        detected_cams = dai.Device.getConnectedCameras()
        self.get_logger().info(f'Detected {len(detected_cams)} cameras.')

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Left camera
        self.left_cam = self.pipeline.create(dai.node.Camera)
        self.left_cam.setResolution(dai.CameraProperties.SensorResolution.THE_400_P)
        self.left_out = self.left_cam.requestOutput((FRAME_WIDTH, FRAME_HEIGHT), type=dai.ImgFrame.Type.RGB888p)

        # Right camera
        self.right_cam = self.pipeline.create(dai.node.Camera)
        self.right_cam.setResolution(dai.CameraProperties.SensorResolution.THE_400_P)
        self.right_out = self.right_cam.requestOutput((FRAME_WIDTH, FRAME_HEIGHT), type=dai.ImgFrame.Type.RGB888p)

        # Build device
        self.device = dai.Device(self.pipeline)

        self.get_logger().info('Pipeline deployed.')

    def spin_once(self):
        left_frame = self.left_out.tryGet()
        right_frame = self.right_out.tryGet()

        self.frame_publishers.publish('left', left_frame)
        self.frame_publishers.publish('right', right_frame)


def main():
    rclpy.init()
    node = LuxonisCamDriverNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.spin_once()
    finally:
        node.get_logger().info('Shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
