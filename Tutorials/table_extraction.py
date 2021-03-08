import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy


pcd = o3d.io.read_point_cloud('./box2.ply')
pcd_cluster = copy.deepcopy(pcd)

print(pcd)
o3d.visualization.draw_geometries([pcd])
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)

print("clusterting using DBSCAN")
with o3d.utility.VerbosityContextManager(
    o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd_cluster.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

max_label = labels.max()
print("point cloud has {} clusters".format(max_label + 1))
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0

pcd_cluster.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([pcd_cluster, mesh])
# o3d.visualization.draw_geometries([pcd])

idx = np.where(labels == 7)[0]
pcd_idx = pcd.select_by_index(idx)
o3d.visualization.draw_geometries([pcd_idx])

plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model
inliers_cloud = pcd.select_by_index(inliers)
o3d.visualization.draw_geometries([inliers_cloud])