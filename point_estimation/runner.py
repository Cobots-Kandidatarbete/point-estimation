import open3d as o3d
import copy
import numpy as np


def run():
    pcd = o3d.io.read_point_cloud(
        'point_estimation/data/realsense_testcase2.ply')
    # o3d.visualization.draw_geometries([pcd])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    rotated_pcd = rotate_pc(pcd)
    o3d.visualization.draw_geometries([rotated_pcd, frame])


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
    return copy.deepcopy(pcd).rotate(R)


if __name__ == '__main__':
    run()
