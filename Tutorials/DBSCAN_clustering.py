import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

pcd = o3d.io.read_point_cloud("./fragment.ply")

with o3d.utility.VerbosityContextManager(
    o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

max_label = labels.max()
print("point cloud has {} clusters".format(max_label + 1))
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0  # noise is labeled as -1
print(colors)
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd])