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

# If you’re not explicitly setting board sockets here,
# you may rely on defaults or your board’s wiring/config.
LEFT_CAM_SOCKET = dai.CameraBoardSocket.CAM_A
RIGHT_CAM_SOCKET = dai.CameraBoardSocket.CAM_D

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

    def try_get_publish(self, topic: StreamTopic, queue: dai.MessageQueue) -> None:
        msg = queue.tryGet()
        if msg is None:
            return
        if not isinstance(msg, dai.ImgFrame):
            self.node.get_logger().warn('Dequeued something other than an image frame, skipping')
            return
        time_msg = self.node.get_clock().now().to_msg()
        img_msg = self.get_image_msg(msg.getCvFrame(), time_msg)
        if topic in self.publishers:
            self.publishers[topic].publish(img_msg)
        else:
            self.node.get_logger().warning(f'Invalid camera publisher topic "{topic.value}", not publishing')

    def get_image_msg(self, image: Matlike, time: Time) -> Image:
        inverted_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_msg: Image = self.bridge.cv2_to_imgmsg(inverted_image)
        img_msg.encoding = 'rgb8'
        img_msg.header.stamp = time
        return img_msg

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
            CAM_IDS.LUX_LEFT:   StreamMeta.of('left', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_RIGHT:  StreamMeta.of('right', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_LEFT_RECT:  StreamMeta.of('left_rect', StreamTopic.RECT_LEFT, enabled=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of('right_rect', StreamTopic.RECT_RIGHT, enabled=False),
            CAM_IDS.LUX_DISPARITY:  StreamMeta.of('disparity', StreamTopic.DISPARITY, enabled=False),
            CAM_IDS.LUX_DEPTH:      StreamMeta.of('depth', StreamTopic.DEPTH, enabled=False),
        }

        self.left_stereo_script_topics = StreamScriptTopicSet.of('left_stereo')
        self.right_stereo_script_topics = StreamScriptTopicSet.of('right_stereo')
        self.script_topics = (
            *(meta.script_topics for meta in self.stream_metas.values()),
            self.left_stereo_script_topics,
            self.right_stereo_script_topics,
        )

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )
        self.intrinsics_publishers = (
            self.create_publisher(Intrinsics, 'luxonis_left_intrinsics',   QoSPresetProfiles.DEFAULT.value),
            self.create_publisher(Intrinsics, 'luxonis_right_intrinsics',  QoSPresetProfiles.DEFAULT.value),
        )

        self.deploy_pipeline()

        calib_data = self.device.readCalibration()
        focal_lengths_mm = [0.0, 0.0]
        self.intrinsics: list[list[list[float]]] = []
        try:
            for i, cam in enumerate((LEFT_CAM_SOCKET, RIGHT_CAM_SOCKET)):
                intr = calib_data.getCameraIntrinsics(cam)
                self.intrinsics.append(intr)
                focal_lengths_mm[i] = intr[0][0] * 3 / 1000
            self.get_logger().info(f'Focal lengths: {focal_lengths_mm}')
        except IndexError:
            self.get_logger().warn('Unable to get Luxonis intrinsics. Did you calibrate?')

        self.frame_publishers = FramePublishers(self)
        self.get_logger().info('Pipeline created')
        self.missed_sends = 0

    def cam_manage_callback(
        self, request: CameraManage.Request, response: CameraManage.Response
    ) -> CameraManage.Response:
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

        # Left camera
        left_cam = pipeline.create(dai.node.Camera)
        # No call to setBoardSocket
        # Configure left camera
        left_cam.outputs['rgb'].setStreamName('left_rgb')
        left_cam.outputs['rgb'].setResolution((FRAME_WIDTH, FRAME_HEIGHT))
        left_cam.outputs['rgb'].setInterleaved(False)
        left_cam.outputs['rgb'].setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Right camera
        right_cam = pipeline.create(dai.node.Camera)
        # No call to setBoardSocket
        right_cam.outputs['rgb'].setStreamName('right_rgb')
        right_cam.outputs['rgb'].setResolution((FRAME_WIDTH, FRAME_HEIGHT))
        right_cam.outputs['rgb'].setInterleaved(False)
        right_cam.outputs['rgb'].setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        right_cam.initialControl.setMisc('3a-follow', RIGHT_CAM_SOCKET.value)

        # Script node
        script = pipeline.create(dai.node.Script)
        script_str = f"""
enabled_flags = [False] * {len(self.script_topics)}
toggle_inputs = ["{'", "'.join(names.script_toggle_name for names in self.script_topics)}"]
frame_inputs = ["{'", "'.join(names.script_input_name for names in self.script_topics)}"]
frame_outputs = ["{'", "'.join(names.script_output_name for names in self.script_topics)}"]

while True:
    for i, (toggle_input, frame_input, frame_output) in enumerate(zip(toggle_inputs, frame_inputs, frame_outputs)):
        toggle_msg = node.io[toggle_input].tryGet()
        if toggle_msg is not None:
            enabled_flags[i] = toggle_msg.getData()[0]
        frame = node.io[frame_input].tryGet()
        if frame is not None and enabled_flags[i]:
            node.io[frame_output].send(frame)
"""
        script.setScript(script_str)

        # Link raw streams
        left_cam.outputs['rgb'].link(script.inputs[self.stream_metas[CAM_IDS.LUX_LEFT].script_topics.script_input_name])
        right_cam.outputs['rgb'].link(script.inputs[self.stream_metas[CAM_IDS.LUX_RIGHT].script_topics.script_input_name])

        # Stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        script.outputs[self.left_stereo_script_topics.script_output_name].link(stereo.left)
        script.outputs[self.right_stereo_script_topics.script_output_name].link(stereo.right)

        stereo.rectifiedLeft.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_LEFT_RECT].script_topics.script_input_name]
        )
        stereo.rectifiedRight.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_RIGHT_RECT].script_topics.script_input_name]
        )
        stereo.disparity.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_DISPARITY].script_topics.script_input_name]
        )
        stereo.depth.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_DEPTH].script_topics.script_input_name]
        )

        # Host toggles inputs
        for names in self.script_topics:
            script.inputs[names.script_toggle_name].createInputQueue(maxSize=1, blocking=False)

        # Host outputs: create output queues
        self.frame_output_queues = {}
        for cam_id, meta in self.stream_metas.items():
            q = script.outputs[meta.script_topics.script_output_name].createOutputQueue(maxSize=1, blocking=False)
            self.frame_output_queues[cam_id] = q

        pipeline.start()
        self.device = pipeline  # for calibration reading etc

        self.left_stereo_toggle_queue = script.inputs[self.left_stereo_script_topics.script_toggle_name].createInputQueue(maxSize=1, blocking=False)
        self.right_stereo_toggle_queue = script.inputs[self.right_stereo_script_topics.script_toggle_name].createInputQueue(maxSize=1, blocking=False)
        self.toggle_queues = {
            cam_id: script.inputs[meta.script_topics.script_toggle_name]
            for cam_id, meta in self.stream_metas.items()
        }

        self.get_logger().info('Pipeline deployed')

    def spin(self) -> None:
        if len(self.intrinsics) == len(self.intrinsics_publishers):
            for intr, pub in zip(self.intrinsics, self.intrinsics_publishers, strict=True):
                pub.publish(
                    Intrinsics(
                        fx=intr[0][0],
                        fy=intr[1][1],
                        x0=intr[0][2],
                        y0=intr[1][2],
                        s=intr[0][1],
                    )
                )

        try:
            for cam_id, queue in self.frame_output_queues.items():
                if self.stream_metas[cam_id].enabled:
                    self.frame_publishers.try_get_publish(self.stream_metas[cam_id].topic, queue)

            enable_stereo = any(self.stream_metas[cam_id].enabled for cam_id in STREAMS_THAT_NEED_STEREO)

            buf = dai.Buffer()
            buf.setData([1 if enable_stereo else 0])
            self.left_stereo_toggle_queue.send(buf)
            self.right_stereo_toggle_queue.send(buf)

            for cam_id, input_node in self.toggle_queues.items():
                iq = input_node.createInputQueue(maxSize=1, blocking=False)
                buf2 = dai.Buffer()
                buf2.setData([1 if self.stream_metas[cam_id].enabled else 0])
                iq.send(buf2)

            self.missed_sends = 0
        except RuntimeError:
            self.missed_sends += 1
            self.get_logger().warn('Missed a dual cam spin')

        if self.missed_sends >= MISSED_SENDS_RESET_THRESHOLD:
            self.get_logger().error(
                f'Missed >= {MISSED_SENDS_RESET_THRESHOLD} dual cam spins, redeploying'
            )
            self.deploy_pipeline()
            self.missed_sends = 0

    def shutdown(self) -> None:
        try:
            if hasattr(self.device, 'stop'):
                self.device.stop()
        except Exception:
            pass

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
