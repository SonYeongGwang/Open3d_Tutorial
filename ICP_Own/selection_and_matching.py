import open3d as o3d
import numpy as np

###################### 1. downsample point cloud data ######################

source_path = '/home/a/mouse_data_set/mouse_data_main/mouse_model_ransac.ply'
target_path = '/home/a/mouse_data_set/mouse_data_scene/senthetic/mouse.ply'

source_data = o3d.io.read_point_cloud(source_path) # read point cloud data
target_data = o3d.io.read_point_cloud(target_path)
source_data.paint_uniform_color([1, 0, 0.0])     # paint uniform color
target_data.paint_uniform_color([0.0, 1, 0.0])
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

# visualize inital pcd
source_data.orient_normals_consistent_tangent_plane(10) # make normals oriented to uniform direction
o3d.visualization.draw_geometries([source_data, target_data])

# sample point by random index
sampling_ratio = 0.2
number_of_points = len(np.asarray(source_data.points))
print("The number of input data: ",number_of_points)
indx = np.random.choice(np.arange(0, number_of_points), size=(int(number_of_points * sampling_ratio), ), replace=False)
source_downsampled_random_indx = source_data.select_by_index(indx)
o3d.visualization.draw_geometries([source_downsampled_random_indx])

# sample point by voxelization
source_downsampled_voxel_indx = source_data.voxel_down_sample(voxel_size=0.008)
target_downsampled_voxel_indx = target_data.voxel_down_sample(voxel_size=0.008)
o3d.visualization.draw_geometries([source_downsampled_voxel_indx])

###################### 1. match point cloud data ######################
target_kdtree = o3d.geometry.KDTreeFlann(target_downsampled_voxel_indx)
selection = np.arange(10) # select 10 point in source point

closest_pointset = []
for i in np.arange(0, 10):
    # source_downsampled_voxel_indx.colors[i] = [0, 0, 1]
    [k, idx, _] = target_kdtree.search_knn_vector_3d(np.asarray(source_downsampled_voxel_indx.points)[i], 1)
    closest_point = idx.pop()
    closest_pointset.append(closest_point)
    # target_downsampled_voxel_indx.colors[closest_point] = [0, 0, 1]

print(closest_pointset)
points_source = source_downsampled_voxel_indx.select_by_index(selection)
points_target = target_downsampled_voxel_indx.select_by_index([204, 105, 9, 149, 113, 107, 163, 14, 79, 196])
points_source = np.array(points_source.points)
points_target = np.array(points_target.points)
points = np.concatenate((points_source, points_target), axis=0)

# lines = np.asarray([[0, 1]])
# lines.astype('int32')
# colors = [[1, 0, 0] for i in range(len(lines))]
# line_set = o3d.geometry.LineSet(
#     points=o3d.utility.Vector3dVector(points),
#     lines=o3d.utility.Vector2iVector(lines),
# )
#make linset
# lines = np.asarray([[i, i + len(points)/2]for i in np.arange(len(points)/2)])
lines = np.asarray([[0, 19], [1, 13], [2, 10], [3, 16], [4, 15], [5, 14], [6, 17], [7, 11], [8, 12], [9, 18]])
lines.astype('int32')
colors = [[0, 0, 1] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([source_data, target_data, line_set])