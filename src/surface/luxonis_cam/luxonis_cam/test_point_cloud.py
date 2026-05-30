import time
import depthai as dai
import sys
import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit(
        "Critical dependency missing: Open3D. Please install it using the command: '{} -m pip install open3d' and then rerun the script.".format(
            sys.executable
        )
    )


class O3DNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()

    def run(self) -> None:
        # def key_callback(vis, action, mods):
        #     global isRunning
        #     if action == 0:
        #         isRunning = False

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        # vis.register_key_action_callback(81, key_callback)
        pcd = o3d.geometry.PointCloud()

        try:
            print('before sleep')
            time.sleep(5)
            inPointCloud = self.inputPCL.tryGet()
            print('got inputPCL pointcloud')
            time.sleep(5)
            print('after 2nd sleep')
        except dai.MessageQueue.QueueException:
            print('errored')
            return # Pipeline closed
        if inPointCloud is not None:
            print('in if')
            points, colors = inPointCloud.getPointsRGB()
            print('retrieved points')
            pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
            print('made open 3d points')
            colors = (colors / 255.0).astype(np.float64)
            print('got colors')
            pcd.colors = o3d.utility.Vector3dVector(np.delete(colors, 3, 1))
            print('more open3d points')
            vis.add_geometry(pcd)
            print('added geometry')
            vis.run()
            print('ran')
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
        # coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        #     size=1000, origin=[0, 0, 0]
        # )
        # vis.add_geometry(coordinateFrame)
        # first = True
        # while True:
        #     try:
        #         inPointCloud = self.inputPCL.tryGet()
        #     except dai.MessageQueue.QueueException:
        #         return # Pipeline closed
        #     if inPointCloud is not None:
        #         points, colors = inPointCloud.getPointsRGB()
        #         pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        #         colors = (colors / 255.0).astype(np.float64)
        #         pcd.colors = o3d.utility.Vector3dVector(np.delete(colors, 3, 1))
        #         if first:
        #             vis.add_geometry(pcd)
        #             first = False
        #         else:
        #             vis.update_geometry(pcd)
        #     vis.poll_events()
        #     vis.update_renderer()

        # vis.destroy_window()
        # print(vis.get_picked_points())

# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    # Define sources and outputs
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
    # stereo.initialConfig.setConfidenceThreshold(200)
    # stereo.setLeftRightCheck(True)
    stereo.setSubpixelFractionalBits(3)
    rgbd = p.create(dai.node.RGBD).build()
    align = None
    o3dViewer = p.create(O3DNode)
    left.build(dai.CameraBoardSocket.CAM_A)
    right.build(dai.CameraBoardSocket.CAM_D)
    out = None

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    # stereo.setAlphaScaling(1)

    # Linking
    left.requestOutput((640, 400)).link(stereo.left)
    right.requestOutput((640, 400)).link(stereo.right)
    platform = p.getDefaultDevice().getPlatform()
    if platform == dai.Platform.RVC4:
        out = left.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
        align = p.create(dai.node.ImageAlign)
        stereo.depth.link(align.input)
        out.link(align.inputAlignTo)
        align.outputAligned.link(rgbd.inDepth)
    else:
        out = left.requestOutput(
            (640, 400), dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True
        )
        stereo.depth.link(rgbd.inDepth)
        out.link(stereo.inputAlignTo)
    out.link(rgbd.inColor)

    rgbd.pcl.link(o3dViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
