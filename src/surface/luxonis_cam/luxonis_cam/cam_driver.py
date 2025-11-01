import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage
from dataclasses import dataclass
from enum import StrEnum
from numpy.typing import NDArray
from numpy import generic


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
class StreamMeta:
    topic: StreamTopic
    enabled: bool

    @staticmethod
    def of(topic: StreamTopic, enabled: bool) -> 'StreamMeta':
        return StreamMeta(topic=topic, enabled=enabled)


CAM_IDS = CameraManage.Request


STREAMS_THAT_NEED_STEREO = [
    CAM_IDS.LUX_LEFT_RECT,
    CAM_IDS.LUX_RIGHT_RECT,
    CAM_IDS.LUX_DISPARITY,
    CAM_IDS.LUX_DEPTH,
]


class FramePublishers:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.bridge = CvBridge()
        self.publishers = {
            topic: node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)
            for topic in StreamTopic
        }

    def publish_frame(self, topic: StreamTopic, frame: dai.ImgFrame) -> None:
        img = frame.getCvFrame()
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        time_msg = self.node.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
        img_msg.header.stamp = time_msg
        self.publishers[topic].publish(img_msg)


class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver')

        self.stream_metas = {
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of(StreamTopic.RECT_LEFT, enabled=True),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of(StreamTopic.RECT_RIGHT, enabled=True),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of(StreamTopic.DISPARITY, enabled=True),
            CAM_IDS.LUX_DEPTH: StreamMeta.of(StreamTopic.DEPTH, enabled=True),
        }

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )

        self.intrinsics_publishers = (
            self.create_publisher(Intrinsics, 'luxonis_left_intrinsics', QoSPresetProfiles.DEFAULT.value),
            self.create_publisher(Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value),
        )

        self.frame_publishers = FramePublishers(self)

        self.deploy_pipeline()
        self.missed_sends = 0

    def deploy_pipeline(self) -> None:
        """Create and deploy DepthAI v3 pipeline"""
        pipeline = dai.Pipeline()

        # Camera setup
        left_cam = pipeline.create(dai.node.Camera)
        right_cam = pipeline.create(dai.node.Camera)

        left_cam.setCamera("left")
        right_cam.setCamera("right")
        left_cam.setResolution(dai.CameraProperties.SensorResolution.THE_400_P)
        right_cam.setResolution(dai.CameraProperties.SensorResolution.THE_400_P)
        left_cam.setSize(FRAME_WIDTH, FRAME_HEIGHT)
        right_cam.setSize(FRAME_WIDTH, FRAME_HEIGHT)

        # Stereo node
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        left_cam.out.link(stereo.left)
        right_cam.out.link(stereo.right)

        # Deploy to device before querying connected cameras
        self.device = dai.Device(pipeline)
        detected_cams = self.device.getConnectedCameras()
        self.get_logger().info(f"Detected cameras: {detected_cams}")

        # Output queues
        self.frame_output_queues = {
            CAM_IDS.LUX_LEFT_RECT: stereo.rectifiedLeft.createOutputQueue(maxSize=1, blocking=False),
            CAM_IDS.LUX_RIGHT_RECT: stereo.rectifiedRight.createOutputQueue(maxSize=1, blocking=False),
            CAM_IDS.LUX_DISPARITY: stereo.disparity.createOutputQueue(maxSize=1, blocking=False),
            CAM_IDS.LUX_DEPTH: stereo.depth.createOutputQueue(maxSize=1, blocking=False),
        }

        # Get intrinsics (if available)
        calib = self.device.readCalibration()
        self.intrinsics = []
        try:
            for i, cam in enumerate(["left", "right"]):
                intr = calib.getCameraIntrinsics(cam)
                self.intrinsics.append(intr)
                fx_mm = intr[0][0] * 3 / 1000
                self.get_logger().info(f"{cam} fx (mm): {fx_mm}")
        except Exception as e:
            self.get_logger().warn(f"Could not read intrinsics: {e}")

        self.get_logger().info("Pipeline deployed to DepthAI v3 device")

    def cam_manage_callback(self, request: CameraManage.Request, response: CameraManage.Response):
        if request.cam in self.stream_metas:
            self.stream_metas[request.cam].enabled = request.on
            response.success = True
        else:
            response.success = False
        statuses = [f"{cam}: {meta.enabled}" for cam, meta in self.stream_metas.items()]
        self.get_logger().info("Luxonis stream states: " + "; ".join(statuses))
        return response

    def spin(self) -> None:
        """Publish camera frames"""
        try:
            for cam_id, q in self.frame_output_queues.items():
                if self.stream_metas[cam_id].enabled:
                    frame = q.tryGet()
                    if frame:
                        topic = self.stream_metas[cam_id].topic
                        self.frame_publishers.publish_frame(topic, frame)
            self.missed_sends = 0
        except RuntimeError as e:
            self.missed_sends += 1
            self.get_logger().warn(f"Frame grab failed: {e}")

        if self.missed_sends >= MISSED_SENDS_RESET_THRESHOLD:
            self.get_logger().error("Too many missed sends — redeploying pipeline")
            self.deploy_pipeline()
            self.missed_sends = 0

    def shutdown(self) -> None:
        if hasattr(self, "device"):
            self.device.close()


def main() -> None:
    rclpy.init()
    node = LuxonisCamDriverNode()
    executor = MultiThreadedExecutor()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, executor=executor, timeout_sec=0)
            node.spin()
    finally:
        node.shutdown()
