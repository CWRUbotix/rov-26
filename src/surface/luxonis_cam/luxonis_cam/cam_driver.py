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

MISSED_SENDS_RESET_THRESHOLD = 5
FRAME_WIDTH = 640
FRAME_HEIGHT = 400


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
    """Manages publishing DepthAI frames as ROS2 Images."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in StreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: StreamTopic) -> Publisher:
        return self.node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)

    def try_get_publish(self, topic: StreamTopic, queue) -> None:
        packet = queue.tryGet()
        if packet is None:
            return

        if not isinstance(packet, dai.ImgFrame):
            self.node.get_logger().warn(f'Skipped non-image packet on {topic.value}')
            return

        frame = packet.getCvFrame()
        time_msg = self.node.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding='rgb8')
        img_msg.header.stamp = time_msg
        self.publishers[topic].publish(img_msg)


STREAMS_THAT_NEED_STEREO = [
    CAM_IDS.LUX_LEFT_RECT,
    CAM_IDS.LUX_RIGHT_RECT,
    CAM_IDS.LUX_DISPARITY,
    CAM_IDS.LUX_DEPTH,
]


class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver')

        self.stream_metas = {
            CAM_IDS.LUX_LEFT: StreamMeta.of('left', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_RIGHT: StreamMeta.of('right', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect', StreamTopic.RECT_LEFT, enabled=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of('right_rect', StreamTopic.RECT_RIGHT, enabled=False),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity', StreamTopic.DISPARITY, enabled=False),
            CAM_IDS.LUX_DEPTH: StreamMeta.of('depth', StreamTopic.DEPTH, enabled=False),
        }

        self.cam_manage_service = self.create_service(CameraManage, 'manage_luxonis', self.cam_manage_callback)
        self.intrinsics_publishers = (
            self.create_publisher(Intrinsics, 'luxonis_left_intrinsics', QoSPresetProfiles.DEFAULT.value),
            self.create_publisher(Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value),
        )

        self.deploy_pipeline()

        self.frame_publishers = FramePublishers(self)
        self.missed_sends = 0

    def cam_manage_callback(self, request, response):
        response.success = True
        if request.cam in self.stream_metas:
            self.stream_metas[request.cam].enabled = request.on
        else:
            response.success = False

        statuses = [f'{cam}: {meta.enabled}' for cam, meta in self.stream_metas.items()]
        self.get_logger().info(f'Luxonis publishing: {"; ".join(statuses)}')
        return response

    def deploy_pipeline(self) -> None:
        pipeline = dai.Pipeline()

        # Auto-detect left/right cameras
        detected_cams = dai.Device.getConnectedCameras()
        self.get_logger().info(f"Detected cameras: {detected_cams}")

        left_cam = pipeline.create(dai.node.Camera)
        right_cam = pipeline.create(dai.node.Camera)

        left_cam.setCamera("left")
        right_cam.setCamera("right")
        left_cam.setResolution(dai.CameraProperties.SensorResolution.THE_400_P)
        right_cam.setResolution(dai.CameraProperties.SensorResolution.THE_400_P)
        left_cam.setSize(FRAME_WIDTH, FRAME_HEIGHT)
        right_cam.setSize(FRAME_WIDTH, FRAME_HEIGHT)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        left_cam.out.link(stereo.left)
        right_cam.out.link(stereo.right)

        # Queues (automatic host bridge)
        self.frame_output_queues = {}
        outputs = {
            CAM_IDS.LUX_LEFT_RECT: stereo.rectifiedLeft,
            CAM_IDS.LUX_RIGHT_RECT: stereo.rectifiedRight,
            CAM_IDS.LUX_DISPARITY: stereo.disparity,
            CAM_IDS.LUX_DEPTH: stereo.depth,
        }

        for cam_id, out in outputs.items():
            q = out.createOutputQueue(maxSize=1, blocking=False)
            self.frame_output_queues[cam_id] = q

        self.device = pipeline.startDevice()
        self.get_logger().info("Pipeline deployed to DepthAI v3 device")

    def spin(self):
        try:
            for cam_id, q in self.frame_output_queues.items():
                if self.stream_metas[cam_id].enabled:
                    self.frame_publishers.try_get_publish(self.stream_metas[cam_id].topic, q)

        except RuntimeError as e:
            self.missed_sends += 1
            self.get_logger().warn(f'Missed frame send: {e}')

        if self.missed_sends >= MISSED_SENDS_RESET_THRESHOLD:
            self.get_logger().error(f'Missed {MISSED_SENDS_RESET_THRESHOLD} spins, redeploying')
            self.deploy_pipeline()
            self.missed_sends = 0

    def shutdown(self):
        if hasattr(self, 'device') and self.device:
            self.device.close()


def main():
    rclpy.init()
    node = LuxonisCamDriverNode()
    executor = MultiThreadedExecutor()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, executor=executor, timeout_sec=0)
            node.spin()
    finally:
        node.shutdown()
