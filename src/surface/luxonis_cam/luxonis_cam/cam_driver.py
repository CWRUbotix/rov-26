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
    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in StreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: StreamTopic) -> Publisher:
        return self.node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)

    def try_get_publish(self, topic: StreamTopic, img_frame: dai.ImgFrame) -> None:
        if img_frame is None:
            return
        # Type check
        if not isinstance(img_frame, dai.ImgFrame):
            self.node.get_logger().warn(f'Dequeued something other than an image frame on topic {topic.value}, skipping')
            return
        time_msg = self.node.get_clock().now().to_msg()
        cv_img = img_frame.getCvFrame()
        # convert BGR->RGB
        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        img_msg: Image = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
        img_msg.header.stamp = time_msg
        if topic in self.publishers:
            self.publishers[topic].publish(img_msg)
        else:
            self.node.get_logger().warning(f'Invalid camera publisher topic "{topic.value}", not publishing')


STREAMS_THAT_NEED_STEREO = [
    CAM_IDS.LUX_LEFT_RECT,
    CAM_IDS.LUX_RIGHT_RECT,
    CAM_IDS.LUX_DISPARITY,
    CAM_IDS.LUX_DEPTH,
]


class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.stream_metas = {
            CAM_IDS.LUX_LEFT:      StreamMeta.of('left',       StreamTopic.LUX_RAW,     enabled=False),
            CAM_IDS.LUX_RIGHT:     StreamMeta.of('right',      StreamTopic.LUX_RAW,     enabled=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect',  StreamTopic.RECT_LEFT,   enabled=False),
            CAM_IDS.LUX_RIGHT_RECT:StreamMeta.of('right_rect', StreamTopic.RECT_RIGHT,  enabled=False),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity',   StreamTopic.DISPARITY,  enabled=False),
            CAM_IDS.LUX_DEPTH:     StreamMeta.of('depth',       StreamTopic.DEPTH,      enabled=False),
        }

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )
        self.intrinsics_publishers = (
            self.create_publisher(Intrinsics, 'luxonis_left_intrinsics',  QoSPresetProfiles.DEFAULT.value),
            self.create_publisher(Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value),
        )

        self.frame_publishers = FramePublishers(self)

        self.device = None
        self.frame_nodes = {}
        self.intrinsics = []

        self.deploy_pipeline()
        self.missed_sends = 0

    def cam_manage_callback(self,
                            request: CameraManage.Request,
                            response: CameraManage.Response) -> CameraManage.Response:
        response.success = True
        if request.cam in self.stream_metas:
            self.stream_metas[request.cam].enabled = request.on
        else:
            response.success = False

        statuses = [f'{cam}: {meta.enabled}' for cam, meta in self.stream_metas.items()]
        self.get_logger().info(f'Luxonis now publishing: {"; ".join(statuses)}')
        return response

    def deploy_pipeline(self) -> None:
        """Create and deploy DepthAI v3 pipeline."""
        pipeline = dai.Pipeline()

        # Create camera nodes
        left_cam = pipeline.create(dai.node.Camera)
        right_cam = pipeline.create(dai.node.Camera)

        # Request outputs
        left_out = left_cam.requestOutput((FRAME_WIDTH, FRAME_HEIGHT),
                                          type=dai.ImgFrame.Type.RGB888p)
        right_out = right_cam.requestOutput((FRAME_WIDTH, FRAME_HEIGHT),
                                           type=dai.ImgFrame.Type.RGB888p)

        # Stereo depth node
        stereo = pipeline.create(dai.node.SteroDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        # Link camera outputs to stereo inputs
        left_out.link(stereo.left)
        right_out.link(stereo.right)

        # Map frame nodes
        self.frame_nodes = {
            CAM_IDS.LUX_LEFT: left_out,
            CAM_IDS.LUX_RIGHT: right_out,
            CAM_IDS.LUX_LEFT_RECT: stereo.rectifiedLeft,
            CAM_IDS.LUX_RIGHT_RECT: stereo.rectifiedRight,
            CAM_IDS.LUX_DISPARITY: stereo.disparity,
            CAM_IDS.LUX_DEPTH: stereo.depth,
        }

        # Deploy device
        self.device = dai.Device(pipeline)
        connected = self.device.getConnectedCameras()  # correct method usage :contentReference[oaicite:1]{index=1}
        self.get_logger().info(f"Connected cameras: {connected}")

        # Create output queues for each stream
        for cam_id, node_out in self.frame_nodes.items():
            # create host queue
            q = node_out.createOutputQueue(maxSize=1, blocking=False)
            self.frame_nodes[cam_id] = q

        # Read calibration/intrinsics
        calib = self.device.readCalibration()
        try:
            for cam_socket in connected:
                intr = calib.getCameraIntrinsics(cam_socket)
                self.intrinsics.append(intr)
            self.get_logger().info(f'Intrinsics found for {len(self.intrinsics)} cameras')
        except Exception as e:
            self.get_logger().warn(f'Unable to get intrinsics: {e}')

        self.get_logger().info('Pipeline deployed.')

    def spin(self) -> None:
        if self.device is None:
            return

        try:
            for cam_id, queue in self.frame_nodes.items():
                if self.stream_metas.get(cam_id, None) and self.stream_metas[cam_id].enabled:
                    frame = queue.tryGet()
                    if frame:
                        self.frame_publishers.try_get_publish(self.stream_metas[cam_id].topic, frame)
            self.missed_sends = 0
        except RuntimeError as e:
            self.missed_sends += 1
            self.get_logger().warn(f'Missed a frame iteration: {e}')

        if self.missed_sends >= MISSED_SENDS_RESET_THRESHOLD:
            self.get_logger().error(f'Missed >= {MISSED_SENDS_RESET_THRESHOLD} frames, redeploying pipeline')
            self.deploy_pipeline()
            self.missed_sends = 0

    def shutdown(self) -> None:
        if self.device:
            self.device.close()


def main() -> None:
    rclpy.init()
    driver_node = LuxonisCamDriverNode()
    executor = MultiThreadedExecutor()

    try:
        while rclpy.ok():
            rclpy.spin_once(driver_node, executor=executor, timeout_sec=0)
            driver_node.spin()
    finally:
        driver_node.shutdown()
