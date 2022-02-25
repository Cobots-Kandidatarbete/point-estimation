import open3d as o3d
import copy
import numpy as np
import matplotlib.pyplot as plt
from probreg import cpd


def run():
    pcd = o3d.io.read_point_cloud(
        'point_estimation/data/realsense_testcase2.ply')
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    rotated_pcd, plane_model, inliers = rotate_pc(pcd)
    rotated_pcd = rotated_pcd.select_by_index(inliers, invert=True)

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            rotated_pcd.cluster_dbscan(eps=0.1, min_points=100, print_progress=True))

    max_label = labels.max()

    clusters = []

    if max_label > -1:
        for i in range(0, max_label+1):
            cluster_index = np.where(labels == i)[0]
            clusters.append(rotated_pcd.select_by_index(cluster_index))

    clusters[-1] = clusters[-1].random_down_sample(
        sampling_ratio=1000/len(np.asarray(clusters[-1].points)))

    box = o3d.io.read_point_cloud('point_estimation/data/box_750.ply')
    box = box.random_down_sample(
        sampling_ratio=1000/len(np.asarray(box.points)))

    tf_param, _, _ = cpd.registration_cpd(box, clusters[-1])
    box.points = tf_param.transform(box.points)

    clusters[-1].paint_uniform_color([1, 0, 0])
    box.paint_uniform_color([1, 0, 0])
    rotated_pcd.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([box, rotated_pcd])


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
