import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("/home/a/Open3d_Tutorial/ICP/cloud_bin_0.pcd")
pcd_0 = pcd.select_by_index([0])
xyz = np.asarray(pcd.points)

print(np.asarray(pcd_0.points), xyz[0])

def select_by_index(pcd, index=[]):
    assert len(index) != 0

    xyz = o3d.geometry.PointCloud()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    colors = np.asarray(pcd.colors)

    xyz.points = o3d.utility.Vector3dVector(points[index])
    xyz.normals = o3d.utility.Vector3dVector(normals[index])
    xyz.colors = o3d.utility.Vector3dVector(colors[index])

    return xyz

index = [i for i in range(500)]
pcd_new = select_by_index(pcd, index=index)
ground_trh = pcd.select_by_index(index)

o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd_new])
o3d.visualization.draw_geometries([ground_trh])