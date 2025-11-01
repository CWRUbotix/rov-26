from dataclasses import dataclass
from enum import StrEnum

import cv2
import depthai as dai
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from numpy import generic
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage

Matlike = NDArray[generic]

FRAME_WIDTH = 640
FRAME_HEIGHT = 400

MISSED_SENDS_RESET_THRESHOLD = 5

class StreamTopic(StrEnum):
    LUX_RAW = 'lux_raw/image_raw'
    RECT_LEFT = 'rect_left/image_raw'
    RECT_RIGHT = 'rect_right/image_raw'
    DISPARITY = 'disparity/image_raw'
    DEPTH = 'depth/image_raw'

@dataclass
class StreamScriptTopicSet:
    toggle_in_stream_name: str
    script_toggle_name: str
    script_input_name: str
    script_output_name: str

    @staticmethod
    def of(stream_name: str) -> 'StreamScriptTopicSet':
        return StreamScriptTopicSet(
            toggle_in_stream_name=f'{stream_name}_toggle_in',
            script_toggle_name=f'{stream_name}_toggle',
            script_input_name=f'{stream_name}_script_in',
            script_output_name=f'{stream_name}_script_out',
        )

@dataclass
class StreamMeta:
    topic: StreamTopic
    script_topics: StreamScriptTopicSet
    out_stream_name: str
    enabled: bool

    @staticmethod
    def of(stream_name: str, topic: StreamTopic, *, enabled: bool) -> 'StreamMeta':
        return StreamMeta(
            topic=topic,
            script_topics=StreamScriptTopicSet.of(stream_name),
            out_stream_name=f'{stream_name}_out',
            enabled=enabled,
        )

CAM_IDS = CameraManage.Request

class FramePublishers:
    """Singleton to manage publishing video frames."""
    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in StreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: StreamTopic) -> Publisher:
        return self.node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)

    def publish_frame(self, topic: StreamTopic, frame: Matlike) -> None:
        time_msg = self.node.get_clock().now().to_msg()
        img_msg: Image = self.bridge.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 'rgb8')
        img_msg.header.stamp = time_msg
        if topic in self.publishers:
            self.publishers[topic].publish(img_msg)
        else:
            self.node.get_logger().warning(f'Invalid camera publisher topic "{topic.value}"')

class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.stream_metas = {
            CAM_IDS.LUX_LEFT: StreamMeta.of('left', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_RIGHT: StreamMeta.of('right', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect', StreamTopic.RECT_LEFT, enabled=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of('right_rect', StreamTopic.RECT_RIGHT, enabled=False),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity', StreamTopic.DISPARITY, enabled=False),
            CAM_IDS.LUX_DEPTH: StreamMeta.of('depth', StreamTopic.DEPTH, enabled=False),
        }

        self.frame_publishers = FramePublishers(self)

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )
        self.intrinsics_publishers = (
            self.create_publisher(Intrinsics, 'luxonis_left_intrinsics', QoSPresetProfiles.DEFAULT.value),
            self.create_publisher(Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value),
        )

        self.device = None
        self.frame_nodes = {}
        self.deploy_pipeline()
        self.missed_sends = 0

    def cam_manage_callback(self, request: CameraManage.Request, response: CameraManage.Response) -> CameraManage.Response:
        response.success = True
        if request.cam in self.stream_metas:
            self.stream_metas[request.cam].enabled = request.on
        else:
            response.success = False

        statuses = [f'{cam}: {meta.enabled}' for cam, meta in self.stream_metas.items()]
        self.get_logger().info(f'Luxonis now publishing: {"; ".join(statuses)}')
        return response

    def deploy_pipeline(self) -> None:
        pipeline = dai.Pipeline()

        # Create cameras
        left_cam = pipeline.create(dai.node.Camera).build()
        right_cam = pipeline.create(dai.node.Camera).build()

        # Request RGB outputs
        left_out = left_cam.requestOutput((FRAME_WIDTH, FRAME_HEIGHT), type=dai.ImgFrame.Type.RGB888p)
        right_out = right_cam.requestOutput((FRAME_WIDTH, FRAME_HEIGHT), type=dai.ImgFrame.Type.RGB888p)

        # Stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)
        left_out.link(stereo.left)
        right_out.link(stereo.right)

        # Store frame nodes for later spin
        self.frame_nodes[CAM_IDS.LUX_LEFT] = left_out
        self.frame_nodes[CAM_IDS.LUX_RIGHT] = right_out
        self.frame_nodes[CAM_IDS.LUX_LEFT_RECT] = stereo.rectifiedLeft
        self.frame_nodes[CAM_IDS.LUX_RIGHT_RECT] = stereo.rectifiedRight
        self.frame_nodes[CAM_IDS.LUX_DISPARITY] = stereo.disparity
        self.frame_nodes[CAM_IDS.LUX_DEPTH] = stereo.depth

        # Deploy pipeline to device
        self.device = dai.Device(pipeline)

        # Try to read intrinsics
        calib_data = self.device.readCalibration()
        self.intrinsics = []
        try:
            for cam_index, cam_socket in enumerate((0, 1)):  # Use 0/1 as left/right index
                self.intrinsics.append(calib_data.getCameraIntrinsics(cam_socket))
            self.get_logger().info(f'Focal lengths: {[self.intrinsics[0][0][0], self.intrinsics[1][1][1]]}')
        except Exception:
            self.get_logger().warn('Unable to get Luxonis intrinsics. Did you calibrate?')

        self.get_logger().info('Pipeline deployed')

    def spin(self) -> None:
        if self.device is None:
            return

        for cam_id, frame_node in self.frame_nodes.items():
            if self.stream_metas[cam_id].enabled:
                frame = frame_node.tryGet()
                if frame is not None:
                    cv_frame = frame.getCvFrame()
                    self.frame_publishers.publish_frame(self.stream_metas[cam_id].topic, cv_frame)

    def shutdown(self) -> None:
        if self.device:
            self.device.close()

def main() -> None:
    rclpy.init()
    driver_node = LuxonisCamDriverNode()
    executor = MultiThreadedExecutor()

    try:
        while True:
            rclpy.spin_once(driver_node, executor=executor, timeout_sec=0)
            driver_node.spin()
    finally:
        driver_node.shutdown()
