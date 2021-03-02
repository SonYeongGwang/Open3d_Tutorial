import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud('./eagle.ply')
pcd = pcd.voxel_down_sample(voxel_size=0.2)
o3d.visualization.draw_geometries([pcd])
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

print(pcd)
print(type(pcd))
print(type(pcd_tree))

pcd.colors[500] = [0, 1, 0]
o3d.visualization.draw_geometries([pcd])

'''
Using search knn vector 3d
'''
print("Find its 200 nearest neighbors, and paint them blue")
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[500], 20)
# print(k, type(k), idx, type(np.asarray(idx)))

# print(np.asarray(idx))
for i in np.asarray(idx):
    pcd.colors[i] = [0, 0, 1]

o3d.visualization.draw_geometries([pcd])

'''
Using search radius vector 3d
'''
print("Find its neighbors with distance less than 0.2 and paint them green")
[k2, idx2, _] = pcd_tree.search_radius_vector_3d(pcd.points[500], 0.6)

for i in np.asarray(idx2):
    pcd.colors[i] = [1, 0, 0]

o3d.visualization.draw_geometries([pcd])