import open3d as o3d


def run():
    pcd = o3d.io.read_point_cloud(
        'point_estimation/data/realsense_testcase2.ply')
    o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    run()
