import open3d as o3d
import copy
import numpy as np
import matplotlib.pyplot as plt


def run():
    pcd = o3d.io.read_point_cloud(
        'point_estimation/data/realsense_testcase2.ply')
    # o3d.visualization.draw_geometries([pcd])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    rotated_pcd, plane_model, inliers = rotate_pc(pcd)
    rotated_pcd = rotated_pcd.select_by_index(inliers, invert=True)

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            rotated_pcd.cluster_dbscan(eps=0.1, min_points=100, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(
        labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    rotated_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([rotated_pcd, pcd, frame])


def rotate_pc(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                             ransac_n=3,
                                             num_iterations=1000)
    plane_normal = np.array(plane_model[0:-1])
    norm_normal = plane_normal/np.linalg.norm(plane_normal)
    z_axis = np.array([0, 0, 1])
    cross = np.cross(norm_normal, z_axis)
    c = np.dot(norm_normal, z_axis)
    v = np.matrix([[0, -cross[2], cross[1]], [cross[2],
                  0, -cross[0]], [-cross[1], cross[0], 0]])
    R = np.eye(3) + v + np.matmul(v, v)*1/(1+c)
    return [copy.deepcopy(pcd).rotate(R), plane_model, inliers]


if __name__ == '__main__':
    run()
