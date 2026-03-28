# mypy: ignore-errors

import copy
import sys
import time

import depthai as dai
import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit(
        f"Critical dependency missing: Open3D. Please install it using the command: '{sys.executable} -m pip install open3d' and then rerun the script."  # noqa: E501
    )

point_clouds = []

def draw_registration_result(source, target, transformation) -> None:  # noqa: ANN001
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    print('after copy point clouds')

    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])

    print('after paint uniform')

    source_temp.transform(transformation)

    print('after transform')

    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
    print('after visualization')

class O3DNode(dai.node.ThreadedHostNode):
    def __init__(self) -> None:
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()

    def run(self) -> None:
        def key_callback(vis, action, mods) -> None:  # noqa: ANN001, ARG001
            global isRunning  # noqa: PLW0603
            if action == 0:
                isRunning = False

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.register_key_action_callback(81, key_callback)
        pcd = o3d.geometry.PointCloud()

        coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1000, origin=[0, 0, 0]
        )
        vis.add_geometry(coordinateFrame)
        first = True
        while len(point_clouds) < 2:
            time.sleep(1)
            try:
                inPointCloud = self.inputPCL.tryGet()
            except dai.MessageQueue.QueueException:
                return # Pipeline closed
            if inPointCloud is not None:
                points, colors = inPointCloud.getPointsRGB()
                pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
                colors = (colors / 255.0).astype(np.float64)
                pcd.colors = o3d.utility.Vector3dVector(np.delete(colors, 3, 1))
                if first:
                    vis.add_geometry(pcd)
                    first = False
                else:
                    vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            keep = input("Keep this point cloud? (y/n) ")

            if keep == 'y':
                point_clouds.append(copy.deepcopy(pcd))
                print('Point cloud taken')
            else:
                print('Trying different point cloud')

        target = point_clouds[0]
        target.estimate_normals()
        source = point_clouds[1]

        # trans_init = np.identity(4)

        # r = o3d.geometry.get_rotation_matrix_from_xyz((0, np.radians(-45), 0))

        # trans_init = np.eye(4)
        # trans_init[:3, :3] = r


        print("finished finding normals")
        voxel_size = 0.05
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)


        print("after voxels")
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        print("after source_down normals")

        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(source_down,
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(target_down,
                    o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))
        
        print("after fpfh")

        distance_threshold = 0.01

        print("after distance threshold")
        result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.4)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
        )

        print("after ransac")

        trans_init = result_ransac.transformation
        print("after trans init")

        # draw_registration_result(source, target, trans_init)
        threshold = 0.02
        print('Initial alignment')
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, threshold, trans_init)
        print(evaluation)

        threshold=0.02
        print('Apply point-to-point ICP')
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationForColoredICP())
        print(reg_p2p)
        print('Transformation is:')
        print(reg_p2p.transformation)
        draw_registration_result(source, target, reg_p2p.transformation)


        # vis.destroy_window()


# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    # Define sources and outputs
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    #color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()
    align = None
    #color.build()
    o3dViewer = p.create(O3DNode)
    left.build(dai.CameraBoardSocket.CAM_A)
    right.build(dai.CameraBoardSocket.CAM_D)
    out = None

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)

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
