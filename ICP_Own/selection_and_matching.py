import open3d as o3d
import numpy as np

###################### 0. define required function ######################
def argsort(a):
    '''
    Sort index of sorted array with repect to original
    a: numpy like array
    '''
    a = np.array(a)
    a_sort = np.sort(a)
    sorted_indx = []
    for i in np.arange(len(a)):
        interest = a[i]
        indx = np.where(a_sort==interest)
        sorted_indx.append(indx[0][0])
    
    return np.asarray(sorted_indx)

###################### 1. downsample point cloud data ######################

source_path = '/home/a/mouse_data_set/mouse_data_main/mouse_model_ransac.ply'
target_path = '/home/a/mouse_data_set/mouse_data_scene/senthetic/mouse.ply'

source_data = o3d.io.read_point_cloud(source_path) # read point cloud data
target_data = o3d.io.read_point_cloud(target_path)
source_data.paint_uniform_color([1, 0.0, 0.0])     # paint uniform color
target_data.paint_uniform_color([0.0, 1, 0.0])
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

# visualize inital pcd
source_data.orient_normals_consistent_tangent_plane(10) # make normals oriented to uniform direction
# o3d.visualization.draw_geometries([source_data, target_data])

# sample point by random index
sampling_ratio = 0.2
number_of_points = len(np.asarray(source_data.points))
print("The number of input data: ",number_of_points)
indx = np.random.choice(np.arange(0, number_of_points), size=(int(number_of_points * sampling_ratio), ), replace=False)
source_downsampled_random_indx = source_data.select_by_index(indx)
# o3d.visualization.draw_geometries([source_downsampled_random_indx])

# sample point by voxelization
source_downsampled_voxel_indx = source_data.voxel_down_sample(voxel_size=0.008)
target_downsampled_voxel_indx = target_data.voxel_down_sample(voxel_size=0.008)
number_of_points_down = len(np.asarray(source_downsampled_voxel_indx.points))
print("The number of input data after downsaple: ",number_of_points_down)
# o3d.visualization.draw_geometries([source_downsampled_voxel_indx])

###################### 2. match point cloud data ######################
target_kdtree = o3d.geometry.KDTreeFlann(target_downsampled_voxel_indx)
num_of_selection = 21 #len(np.asarray(source_downsampled_voxel_indx.points))
selection = np.arange(num_of_selection) # select 00 point in source point

closest_pointset = []
for i in np.arange(0, num_of_selection):
    # source_downsampled_voxel_indx.colors[i] = [0, 0, 1]
    [k, idx, c] = target_kdtree.search_knn_vector_3d(np.asarray(source_downsampled_voxel_indx.points)[i], 1)
    closest_point = idx.pop()
    closest_pointset.append(closest_point)
    # target_downsampled_voxel_indx.colors[closest_point] = [0, 0, 1]

points_source = source_downsampled_voxel_indx.select_by_index(selection)
points_target = target_downsampled_voxel_indx.select_by_index(closest_pointset)
points_source = np.array(points_source.points)
points_target = np.array(points_target.points)
points = np.concatenate((points_source, points_target), axis=0)
sorting_indx = argsort(closest_pointset)
sorting_indx = sorting_indx + num_of_selection

#make linset

lines = np.asarray([[i, sorting_indx[i]]for i in np.arange(num_of_selection)])
lines.astype('int32')
colors = [[0, 0, 1] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)

line_set.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([source_downsampled_voxel_indx, target_downsampled_voxel_indx, line_set])

###################### 3. align point cloud data #######################
######(for test reason, it doesn't include weighing and rejecting)######
########################################################################

# For alignment, we impliment Least-Squares Fitting
# Ref: Least-Squares Fitting of Two 3-D Point Sets (K. S. ARUN, T. S. HUANG, AND S. D. BLOSTEIN)
# <Algorithm>
# Step 1: make two points set have same centriod
# Step 2: calculate the 3x3 matrix of point set. Let's say the result as H
# Step 3: find SVD(Singular Value Decomposition) of H
# Step 4: calculate X = V*traspose(U)
# Step 5: calculate, det(x), the determinate of X.
#   If det(x) = +1, the R = X.
#   If det(x) = -1, the algorithm fails.(This case usually does not occur.)

source = (np.asarray(source_downsampled_voxel_indx.points[204]))
target = (np.asarray(target_downsampled_voxel_indx.points[0]))
print(np.sqrt(source[0]-target[0])**2 + (source[1]-target[1])**2 + (source[2]-target[2])**2)