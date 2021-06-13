import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

path = '/home/a/Desktop/lab_test.ply'
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
pcd = o3d.io.read_point_cloud(path)
o3d.visualization.draw_geometries([pcd, frame])
points = np.asarray(pcd.points)
indx = np.where((points[:,2] > -0.9))

pcd = pcd.select_by_index(indx[0])
pcd = pcd.voxel_down_sample(voxel_size = 0.005)
xyz = np.asarray(pcd.points)
o3d.visualization.draw_geometries([pcd, frame])

pcd_tree = o3d.geometry.KDTreeFlann(pcd)
avg_dist = []
for i in range(xyz.shape[0]):
    dist = 0
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], 20)

    for j in range(k):
        dist += np.linalg.norm(xyz[idx[0], :] - xyz[idx[j], :])
    
    k_neighbor_dist     = dist/k
    avg_dist.append(k_neighbor_dist)
dist_mean = np.mean(avg_dist)
dist_std = np.std(avg_dist)
alpha = 1
# index_left = np.where(avg_dist >= dist_mean - alpha * dist_std)[0]
# index_right = np.where(avg_dist <= dist_mean + alpha * dist_std)[0]
filterd_index = np.where((avg_dist >= dist_mean - alpha * dist_std) & (avg_dist <= dist_mean + alpha * dist_std))[0]
pcd_filtered = pcd.select_by_index(filterd_index)

pcd_filtered_tree = o3d.geometry.KDTreeFlann(pcd_filtered)
avg_dist_filtered = []
xyz_filtered = np.asarray(pcd_filtered.points)
for i in range(xyz_filtered.shape[0]):
    dist = 0
    [k, idx, _] = pcd_filtered_tree.search_knn_vector_3d(pcd_filtered.points[i], 20)

    for j in range(k):
        dist += np.linalg.norm(xyz_filtered[idx[0], :] - xyz_filtered[idx[j], :])
    
    k_neighbor_dist_filtered     = dist/k
    avg_dist_filtered.append(k_neighbor_dist_filtered)
#===========================plot============================
origianl = plt.bar(np.arange(xyz.shape[0]), avg_dist)
filtered = plt.bar(np.arange(xyz_filtered.shape[0]), avg_dist_filtered)
plt.title("Mean K-nearest neighbors distances")
plt.xlabel("point index")
plt.ylabel("mean distance")
plt.legend(("raw point cloud", "filtered point cloud"))
plt.show()
#===========================================================
o3d.visualization.draw_geometries([pcd_filtered, frame])
