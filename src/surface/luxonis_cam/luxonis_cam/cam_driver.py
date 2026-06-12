from dataclasses import dataclass
from enum import StrEnum

import cv2
import depthai
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from numpy import generic
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage

Matlike = NDArray[generic]

# Stores the calibration
LEFT_CAM_SOCKET = depthai.CameraBoardSocket.CAM_A
RIGHT_CAM_SOCKET = depthai.CameraBoardSocket.CAM_D

MISSED_SENDS_RESET_THRESHOLD = 5

# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800
FRAME_WIDTH = 640
FRAME_HEIGHT = 400


# ROS topics it streams to
class StreamTopic(StrEnum):
    LUX_RAW = 'lux_raw/image_raw'
    RECT_LEFT = 'rect_left/image_raw'
    RECT_RIGHT = 'rect_right/image_raw'
    DISPARITY = 'disparity/image_raw'
    DEPTH = 'depth/image_raw'

class PointStreamTopic(StrEnum):
    POINT_CLOUD = 'rgbd/point_cloud'


@dataclass
class StreamScriptTopicSet:
    """Dataclass representing video stream script topics (toggle/frame I/O stream topics)."""

    script_toggle_name: str
    script_input_name: str
    script_output_name: str

    @staticmethod
    def of(stream_name: str) -> 'StreamScriptTopicSet':
        """
        Create a StreamScriptTopicSet (factory method).

        Parameters
        ----------
        stream_name : str
            name of the stream

        Returns
        -------
        StreamScriptNames
            a dataclass representing stream script topics
        """
        return StreamScriptTopicSet(
            script_toggle_name=f'{stream_name}_toggle',
            script_input_name=f'{stream_name}_script_in',
            script_output_name=f'{stream_name}_script_out',
        )


@dataclass
class StreamMeta:
    """Mutable dataclass representing video stream metadata.

    Holds the ROS topic, StreamScriptTopicSet, and whether the stream is enabled
    by default
    """

    topic: StreamTopic
    script_topics: StreamScriptTopicSet
    enabled: bool

    @staticmethod
    def of(stream_name: str, topic: StreamTopic, *, enabled: bool) -> 'StreamMeta':
        """
        Create a StreamMeta (factory method).

        Parameters
        ----------
        stream_name : str
            name of the stream
        topic : StreamTopic
            ROS topic the stream will be published on
        enabled : bool
            whether the stream is enabled by default

        Returns
        -------
        StreamMeta
            a mutable dataclass representing stream metadata
        """
        return StreamMeta(
            topic=topic,
            script_topics=StreamScriptTopicSet.of(stream_name),
            enabled=enabled,
        )

@dataclass
class PointStreamMeta:
    """Mutable dataclass representing point cloud stream metadata.

    Holds the ROS topic, StreamScriptTopicSet, and whether the stream is enabled
    by default
    """

    topic: PointStreamTopic
    script_topics: StreamScriptTopicSet
    enabled: bool

    @staticmethod
    def of(stream_name: str, topic: PointStreamTopic, *, enabled: bool) -> 'PointStreamMeta':
        """
        Create a StreamMeta (factory method).

        Parameters
        ----------
        stream_name : str
            name of the stream
        topic : StreamTopic
            ROS topic the stream will be published on
        enabled : bool
            whether the stream is enabled by default

        Returns
        -------
        StreamMeta
            a mutable dataclass representing stream metadata
        """
        return PointStreamMeta(
            topic=topic,
            script_topics=StreamScriptTopicSet.of(stream_name),
            enabled=enabled,
        )


# Alias for easier access to LUX_LEFT/LUX_RIGHT/etc.
CAM_IDS = CameraManage.Request


# Takes a ROS node, makes a ROS publisher for each ros topic it is going to use
class FramePublishers:
    """Singleton to manage publishing video frames."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in StreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: StreamTopic) -> Publisher:
        """
        Create a publisher for the specified topic.

        Parameters
        ----------
        topic : StreamTopic
            the topic to publish on

        Returns
        -------
        Publisher
            the new publisher
        """
        return self.node.create_publisher(Image, topic.value, QoSPresetProfiles.DEFAULT.value)

    def try_get_publish(self, topic: StreamTopic, queue: depthai.MessageQueue) -> None:
        """
        Attempt to get a frame from the queue and publish it on the topic.

        Parameters
        ----------
        topic : StreamTopic
            topic to publish to
        queue : depthai.MessageQueue
            queue to read from (single read then give up, won't block long)
        """
        video_frame = queue.tryGet()

        # Discard None (failed to get frame)
        if video_frame is None:
            return

        # Type narrow to make mypy happy
        if not isinstance(video_frame, depthai.ImgFrame):
            self.node.get_logger().warn('Dequeued something other than an image frame, skipping')
            return

        time_msg = self.node.get_clock().now().to_msg()

        if video_frame is not None:
            img_msg = self.get_image_msg(video_frame.getCvFrame(), time_msg)
            if topic in self.publishers:
                self.publishers[topic].publish(img_msg)
            else:
                self.node.get_logger().warning(
                    f'Invalid camera publisher topic "{topic.value}", not publishing'
                )

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
        inverted_image = cv2.cvtColor(image.astype(int), cv2.COLOR_BGR2RGB)
        img_msg: Image = self.bridge.cv2_to_imgmsg(inverted_image)
        img_msg.encoding = 'rgb8'
        img_msg.header.stamp = time
        return img_msg

# Takes a ROS node, makes a ROS publisher for each ros topic it is going to use
class PointFramePublishers:
    """Singleton to manage publishing point cloud frames."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.publishers = {topic: self.make_frame_publisher(topic) for topic in PointStreamTopic}
        self.bridge = CvBridge()

    def make_frame_publisher(self, topic: PointStreamTopic) -> Publisher:
        """
        Create a publisher for the specified topic.

        Parameters
        ----------
        topic : StreamTopic
            the topic to publish on

        Returns
        -------
        Publisher
            the new publisher
        """
        return self.node.create_publisher(PointCloud2, topic.value, QoSPresetProfiles.DEFAULT.value)

    def try_get_publish(self, topic: PointStreamTopic, queue: depthai.MessageQueue) -> None:
        """
        Attempt to get a frame from the queue and publish it on the topic.

        Parameters
        ----------
        topic : StreamTopic
            topic to publish to
        queue : depthai.MessageQueue
            queue to read from (single read then give up, won't block long)
        """
        point_frame = queue.tryGet()

        # Discard None (failed to get frame)
        if point_frame is None:
            return

        # Type narrow to make mypy happy
        if not isinstance(point_frame, depthai.PointCloudData):
            self.node.get_logger().warn('Dequeued something other than a point frame, skipping')
            return

        time_msg = self.node.get_clock().now().to_msg()

        if point_frame is not None:
            point_msg = self.get_point_msg(point_frame, time_msg)
            if topic in self.publishers:
                self.publishers[topic].publish(point_msg)
            else:
                self.node.get_logger().warning(
                    f'Invalid camera publisher topic "{topic.value}", not publishing'
                )

    def get_point_msg(self, point_cloud_data: depthai.PointCloudData, time: Time) -> PointCloud2:
        """Convert cv2 image to ROS2 Image with CvBridge.

        Parameters
        ----------
        image : Matlike
            The image to convert
        time : Time
            The timestamp for the ros message

        Returns
        -------
        PointCloud2
            The ROS2 point cloud message
        """
        points, colors = point_cloud_data.getPointsRGB()

        header = Header()
        header.frame_id = 'frame'
        header.stamp = time

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('r', 12, PointField.UINT8, 1),
            PointField('g', 13, PointField.UINT8, 1),
            PointField('b', 14, PointField.UINT8, 1),
        ]

        msg_points = []

        for point, color in zip(points, colors, strict=False):
            msg_point = point
            msg_point.extend(color[0:3])
            msg_points.append(msg_point)

        point_cloud = point_cloud2.create_cloud(header, fields, msg_points)

        return point_cloud


STREAMS_THAT_NEED_STEREO = [
    CAM_IDS.LUX_LEFT_RECT,
    CAM_IDS.LUX_RIGHT_RECT,
    CAM_IDS.LUX_DISPARITY,
    CAM_IDS.LUX_DEPTH,
]

POINT_STREAMS_THAT_NEED_STEREO = [
    CAM_IDS.POINT_CLOUD,
]

class LuxonisCamDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('luxonis_cam_driver', parameter_overrides=[])

        self.stream_metas = {
            CAM_IDS.LUX_LEFT: StreamMeta.of('left', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_RIGHT: StreamMeta.of('right', StreamTopic.LUX_RAW, enabled=False),
            CAM_IDS.LUX_LEFT_RECT: StreamMeta.of('left_rect', StreamTopic.RECT_LEFT, enabled=False),
            CAM_IDS.LUX_RIGHT_RECT: StreamMeta.of(
                'right_rect', StreamTopic.RECT_RIGHT, enabled=False
            ),
            CAM_IDS.LUX_DISPARITY: StreamMeta.of('disparity', StreamTopic.DISPARITY, enabled=False),
            CAM_IDS.LUX_DEPTH: StreamMeta.of('depth', StreamTopic.DEPTH, enabled=False),
        }

        self.point_stream_metas = {
            CAM_IDS.POINT_CLOUD: PointStreamMeta.of('point_cloud',
                                PointStreamTopic.POINT_CLOUD, enabled=False),
        }

        self.left_stereo_script_topics = StreamScriptTopicSet.of('left_stereo')
        self.right_stereo_script_topics = StreamScriptTopicSet.of('right_stereo')
        self.color_script_topics = StreamScriptTopicSet.of('color')
        self.script_topics = (
            *(meta.script_topics for meta in self.stream_metas.values()),
            *(meta.script_topics for meta in self.point_stream_metas.values()),
            self.left_stereo_script_topics,
            self.right_stereo_script_topics,
            self.color_script_topics,
        )

        self.cam_manage_service = self.create_service(
            CameraManage, 'manage_luxonis', self.cam_manage_callback
        )
        self.intrinsics_publishers = (
            self.create_publisher(
                Intrinsics, 'luxonis_left_intrinsics', QoSPresetProfiles.DEFAULT.value
            ),
            self.create_publisher(
                Intrinsics, 'luxonis_right_intrinsics', QoSPresetProfiles.DEFAULT.value
            ),
        )

        # sets up the input and output queues from the camera
        self.deploy_pipeline()

        calib_data = self.pipeline.getCalibrationData()
        focal_lengths_mm = [0.0, 0.0]
        self.intrinsics: list[list[list[float]]] = []
        try:
            for i, cam in enumerate((LEFT_CAM_SOCKET, RIGHT_CAM_SOCKET)):
                # 3um/px (https://docs.luxonis.com/hardware/sensors/OV9782)
                # / 1000 to get mm
                self.intrinsics.append(calib_data.getCameraIntrinsics(cam))
                focal_lengths_mm[i] = self.intrinsics[-1][0][0] * 3 / 1000
            self.get_logger().info(f'Focal lengths: {focal_lengths_mm}')
        except IndexError:
            self.get_logger().warn('Unable to get Luxonis intrinsics. Did you calibrate?')

        self.frame_publishers = FramePublishers(self)
        self.point_frame_publishers = PointFramePublishers(self)

        self.get_logger().info('Pipeline created')

        self.missed_sends = 0

    def cam_manage_callback(
        self, request: CameraManage.Request, response: CameraManage.Response
    ) -> CameraManage.Response:
        """
        Enable/disable streams based on cam manage service call.

        Parameters
        ----------
        request : CameraManage.Request
            CameraManage service request
        response : CameraManage.Response
            CameraManage service response template

        Returns
        -------
        CameraManage.Response
            the service response
        """
        response.success = True

        if request.cam in self.stream_metas:
            self.stream_metas[request.cam].enabled = request.on
        elif request.cam in self.point_stream_metas:
            self.point_stream_metas[request.cam].enabled = request.on
        else:
            response.success = False

        statuses = [f'{cam}: {meta.enabled}' for cam, meta in self.stream_metas.items()]
        statuses.extend([f'{cam}: {meta.enabled}' for cam, meta in self.point_stream_metas.items()])
        self.get_logger().info(f'Luxonis now publishing: {"; ".join(statuses)}')

        return response

    def deploy_pipeline(self) -> None:
        """Create a depthai pipeline and deploy it to the camera."""
        self.pipeline = depthai.Pipeline()

        left_cam_node = self.pipeline.create(depthai.node.Camera).build(
            LEFT_CAM_SOCKET, sensorResolution=(1280, 800)
        )
        right_cam_node = self.pipeline.create(depthai.node.Camera).build(
            RIGHT_CAM_SOCKET, sensorResolution=(1280, 800)
        )

        right_cam_node.initialControl.setMisc('3a-follow', depthai.CameraBoardSocket.CAM_D.value)

        script = self.pipeline.create(depthai.node.Script)

        # connects left_cam_node and right_cam_node to script inputs
        for node, meta in zip(
            (left_cam_node, right_cam_node),
            [self.stream_metas[cam_id] for cam_id in (CAM_IDS.LUX_LEFT, CAM_IDS.LUX_RIGHT)],
            strict=True,
        ):
            input_name = meta.script_topics.script_input_name
            node.requestOutput(
                (FRAME_WIDTH, FRAME_HEIGHT), type=depthai.ImgFrame.Type.RGB888p
            ).link(script.inputs[input_name])
            script.inputs[input_name].setBlocking(False)
            script.inputs[input_name].setMaxSize(1)

        self.create_toggle_queues(script)

        # Link script outputs to stream_meta outputs
        self.frame_output_queues = {}
        for cam_id, stream_meta in self.stream_metas.items():
            output_name = stream_meta.script_topics.script_output_name
            output_queue = script.outputs[output_name].createOutputQueue(maxSize=1, blocking=False)
            self.frame_output_queues[cam_id] = output_queue

        self.point_output_queues = {}
        for cam_id, stream_meta in self.point_stream_metas.items():
            output_name = stream_meta.script_topics.script_output_name
            output_queue = script.outputs[output_name].createOutputQueue(maxSize=1, blocking=False)
            self.point_output_queues[cam_id] = output_queue

        # Creates lists of which script topics are enabled, where to get the toggle
        # values, where to get the frames from if enabled, and where to output the frames to
        # Loops through script topics and if there is data for the toggle it uses that for enabled
        # If there is data in the frame input and that topic is enabled then it outputs the frame
        script_str = f"""
enabled_flags = [False] * {len(self.script_topics)}
toggle_inputs = ["{'", "'.join([names.script_toggle_name for names in self.script_topics])}"]
frame_inputs = ["{'", "'.join([names.script_input_name for names in self.script_topics])}"]
frame_outputs = ["{'", "'.join([names.script_output_name for names in self.script_topics])}"]

while True:
    for i, (toggle_input, frame_input, frame_output) in enumerate(zip(toggle_inputs, frame_inputs,
                                                                      frame_outputs)):
        toggle_msg = node.inputs[toggle_input].tryGet()
        if toggle_msg is not None:
            enabled_flags[i] = toggle_msg.getData()[0]

        frame = node.inputs[frame_input].tryGet()

        if frame is not None and enabled_flags[i]:
            node.outputs[frame_output].send(frame)
"""
        # self.get_logger().info('\nScript:\n"""' + script_str + '"""\n')
        script.setScript(script_str)

        stereo_node = self.pipeline.create(depthai.node.StereoDepth)
        # unsure what preset mode to use, try this for now
        stereo_node.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DETAIL)

        # Helps with photogrammetry and measurement
        stereo_node.setRectifyEdgeFillColor(0)
        stereo_node.enableDistortionCorrection(arg0=True)

        script.outputs[self.color_script_topics.script_output_name].link(stereo_node.inputAlignTo)

        # Connects the left_cam_node and right_cam_node to be inputs to the script
        left_cam_node.requestFullResolutionOutput().link(
            script.inputs[self.left_stereo_script_topics.script_input_name]
        )
        right_cam_node.requestFullResolutionOutput().link(
            script.inputs[self.right_stereo_script_topics.script_input_name]
        )

        # Get the left camera output to use for color
        left_cam_node.requestOutput((FRAME_WIDTH, FRAME_HEIGHT), depthai.ImgFrame.Type.RGB888i,
            depthai.ImgResizeMode.CROP, enableUndistortion=True).link(
            script.inputs[self.color_script_topics.script_input_name]
        )

        self.deploy_stereo_node(script=script, stereo_node=stereo_node)

        # Node for creating color point clouds
        rgbd = self.pipeline.create(depthai.node.RGBD).build()
        script.outputs[self.stream_metas[CAM_IDS.LUX_DEPTH].script_topics.script_output_name].link(rgbd.inColor)
        script.outputs[self.color_script_topics.script_output_name].link(rgbd.inColor)
        rgbd.pcl.link(script.inputs[self.stream_metas[CAM_IDS.POINT_CLOUD].script_topics.script_input_name])

        self.get_logger().info('Deploying pipeline...')

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

        self.get_logger().info('Pipeline deployed')

    def create_toggle_queues(self, script: depthai.Script) -> None:
        # create toggle input queues
        self.toggle_queues = {}
        for cam_id, meta in self.stream_metas.items():
            toggle_name = meta.script_topics.script_toggle_name
            input_queue = script.inputs[toggle_name].createInputQueue(maxSize=1)
            self.toggle_queues[cam_id] = input_queue

        self.point_toggle_queues = {}
        for cam_id, meta in self.point_stream_metas.items():
            toggle_name = meta.script_topics.script_toggle_name
            input_queue = script.inputs[toggle_name].createInputQueue(maxSize=1)
            self.point_toggle_queues[cam_id] = input_queue

        self.left_stereo_toggle_queue = script.inputs['left_stereo_toggle'].createInputQueue(
            maxSize=1
        )
        self.right_stereo_toggle_queue = script.inputs['right_stereo_toggle'].createInputQueue(
            maxSize=1
        )
        self.color_toggle_queue = script.inputs['color_toggle'].createInputQueue(maxSize=1)

    def deploy_stereo_node(self, stereo_node: depthai.StereoDepth, script: depthai.Script) -> None:
        # Connecting script outputs to the stereo node
        script.outputs[self.left_stereo_script_topics.script_output_name].link(stereo_node.left)
        script.outputs[self.right_stereo_script_topics.script_output_name].link(stereo_node.right)

        stereo_node.rectifiedLeft.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_LEFT_RECT].script_topics.script_input_name]
        )
        stereo_node.rectifiedRight.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_RIGHT_RECT].script_topics.script_input_name]
        )
        stereo_node.disparity.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_DISPARITY].script_topics.script_input_name]
        )
        stereo_node.depth.link(
            script.inputs[self.stream_metas[CAM_IDS.LUX_DEPTH].script_topics.script_input_name]
        )
        self.left_stereo_toggle_queue = script.inputs['left_stereo_toggle_in'].createInputQueue()
        self.right_stereo_toggle_queue = script.inputs['right_stereo_toggle_in'].createInputQueue()

    def spin(self) -> None:
        """Run one iteration of I/O with the Luxonis cam."""
        if len(self.intrinsics) == len(self.intrinsics_publishers):
            # Only publish intrinsics if they've been set (cam is calibrated)
            for intrinsics, publisher in zip(
                self.intrinsics, self.intrinsics_publishers, strict=True
            ):
                publisher.publish(
                    Intrinsics(
                        fx=intrinsics[0][0],
                        fy=intrinsics[1][1],
                        x0=intrinsics[0][2],
                        y0=intrinsics[1][2],
                        s=intrinsics[0][1],
                    )
                )

        try:
            self.publish_frames()
            self.update_toggles()
        except RuntimeError as e:
            self.missed_sends += 1
            self.get_logger().warn('Missed a dual cam spin')
            self.get_logger().warn(e)

        if self.missed_sends >= MISSED_SENDS_RESET_THRESHOLD:
            self.get_logger().error(
                f'Missed >= {MISSED_SENDS_RESET_THRESHOLD} dual cam spins, redeploying'
            )
            self.deploy_pipeline()
            self.missed_sends = 0

        # disparity_frame = self.disparity_queue.tryGet()

        # if disparity_frame:
        #     frame = disparity_frame.getFrame()
        #     frame = (frame * (255 / stereo_node.initialConfig.getMaxDisparity())).astype(uint8)

        #     cv2.imshow('disparity', frame)
        #     frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        #     cv2.imshow('disparity_color', frame)

        #     if cv2.waitKey(1) == ord('q'):
        #         raise KeyboardInterrupt

    def shutdown(self) -> None:
        """Free the device and any other resources."""
        if self.pipeline:
            self.pipeline.__exit__(None, None, None)

    def publish_frames(self) -> None:
        # TODO: only send toggles when we actually need to change state?
        for cam_id, output_queue in self.frame_output_queues.items():
            if self.stream_metas[cam_id].enabled:
                self.frame_publishers.try_get_publish(
                    self.stream_metas[cam_id].topic, output_queue
                )
        for cam_id, output_queue in self.point_output_queues.items():
            if self.point_stream_metas[cam_id].enabled:
                self.point_frame_publishers.try_get_publish(
                    self.point_stream_metas[cam_id].topic, output_queue
                )
                self.get_logger().debug('Published a point cloud')
                self.point_stream_metas[cam_id].enabled = False

    def update_toggles(self) -> None:
        enable_stereo = False
        for cam_id in STREAMS_THAT_NEED_STEREO:
            if self.stream_metas[cam_id].enabled:
                enable_stereo = True
                break

        for cam_id in POINT_STREAMS_THAT_NEED_STEREO:
            if self.point_stream_metas[cam_id].enabled:
                enable_stereo = True
                break

        buf = depthai.Buffer()  # TODO: can we create this once and reuse?
        buf.setData(np.array([1 if enable_stereo else 0], dtype=np.uint8))
        # Send whether the stereo is enabled using the buffer and it toggles the stereo
        self.left_stereo_toggle_queue.send(buf)
        self.right_stereo_toggle_queue.send(buf)
        self.color_toggle_queue.send(buf)

        # Use the toggle queues to send whether each stream meta should be enabled
        for cam_id, toggle_queue in self.toggle_queues.items():
            buf = depthai.Buffer()
            buf.setData(
                np.array([1 if self.stream_metas[cam_id].enabled else 0], dtype=np.uint8)
            )
            toggle_queue.send(buf)

        for cam_id, point_toggle_queue in self.point_toggle_queues.items():
            buf = depthai.Buffer()
            buf.setData(
                np.array([1 if self.point_stream_metas[cam_id].enabled else 0], dtype=np.uint8)
            )
            point_toggle_queue.send(buf)

        self.missed_sends = 0

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
