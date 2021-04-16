import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt

########################## 0. define required function ##########################
def argsort(a):
    '''
    This method is to give order index of sorted array with repect to original 
    and remove effects of same number
    input: a (numpy-like array)
    '''
    a = np.array(a)
    a_sort = np.sort(a)
    sorting_indx = []   # list to store sorting numder(order) of each number
    coin_dic = {}       # dictionary to store information of repeated number
    for interest in a:  # inspect index of sorted array w.r.t original one
        indx = np.where(a_sort==interest)
        sorting_indx.append(indx[0][0])
    
    for num in sorting_indx:    # count the number of repeated amount of each element...(1)
        num_of_coincidence = len(sorting_indx) - np.count_nonzero(np.array(sorting_indx) - num) - 1
        if num_of_coincidence >= 1:
            coin_dic[num] = num_of_coincidence
        else:
            0

    sorting_indx_modi = copy.deepcopy(sorting_indx)

    for coin in coin_dic:       # recalculate sorting_array based on (1) result
        sorting_indx_copy = copy.deepcopy(sorting_indx)
        for i, item in enumerate(sorting_indx_copy - coin):
            if item > 0:
                sorting_indx_modi[i] -= coin_dic[coin]
            else:
                0
    
    return np.array(sorting_indx_modi)

def Visualize(source_pts, target_pts, point_num, closest_set):
    selection = np.arange(0, point_num)
    points_source = source_pts.select_by_index(selection)
    points_target = target_pts.select_by_index(closest_set)
    points_source = np.array(points_source.points)
    points_target = np.array(points_target.points)
    points = np.concatenate((points_source, points_target), axis=0)
    sorting_indx = argsort(closest_set)
    sorting_indx = sorting_indx + point_num
    lines = np.asarray([[i, sorting_indx[i]]for i in selection])
    lines.astype('int32')
    colors = [[0, 0, 1] for i in range(len(lines))]
    line_sets = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )

    line_sets.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([source_pts, target_pts, line_sets])
    return line_sets

def Match(source_pts, target_tree, point_num):
    # target_tree: class 'open3d.cpu.pybind.geometry.KDTreeFlann
    selection = np.arange(point_num) # select (num_of_selection) points in source point
    closest_pointset = []
    for i in np.arange(0, point_num):
        # source_downsampled_voxel_indx.colors[i] = [0, 0, 1]
        [k, idx, c] = target_tree.search_knn_vector_3d(np.asarray(source_pts.points)[i], 1)
        closest_point = idx.pop()
        closest_pointset.append(closest_point)
        # target_downsampled_voxel_indx.colors[closest_point] = [0, 0, 1]

    return closest_pointset


def SelectPointSet(source_idx, source_pts, target_idx, target_pts):
    '''
    This method is to store matched points. Most Important difference with
    'select_by_index' method is that this method includes repeated number in the dataset
    so that total number of points after selection doesn't change
    '''
    sc = []
    tr = []
    [sc.append(np.asarray(source_pts.points[idx])) for idx in source_idx]
    [tr.append(np.asarray(target_pts.points[idx])) for idx in target_idx]

    return np.asarray(sc), np.asarray(tr)

def Align(source_pts, target_pts, point_num, closest_set):
    '''
    For alignment, we impliment Least-Squares Fitting
    Ref: Least-Squares Fitting of Two 3-D Point Sets (K. S. ARUN, T. S. HUANG, AND S. D. BLOSTEIN)
    <Algorithm>
    Step 1: make two points set have same centriod
    Step 2: calculate the 3x3 matrix of point set. Let's say the result as H
    Step 3: find SVD(Singular Value Decomposition) of H
    Step 4: calculate X = V*traspose(U)
    Step 5: calculate, det(x), the determinate of X.
      If det(x) = +1, the R = X.
      If det(x) = -1, the algorithm fails.(This case usually does not occur.)
    output: Rotation, Translation, Homogeneous matrix(Rt, Ts, T)
    '''
    ################# Step 1) #################
    source_mean = np.mean(source_pts, axis=0)
    target_mean = np.mean(target_pts, axis=0)
    
    ################# Step 2) #################
    source_points_cen = source_pts - source_mean
    source_points_cen = np.transpose(source_points_cen)
    target_points_cen = target_pts - target_mean
    H = np.dot(source_points_cen, target_points_cen) # 3x3 matrix

    ################# Step 3) #################
    U, S ,Vt = np.linalg.svd(H) # two matrics are orthogonal
    
    ################# Step 4) #################
    X = np.dot(np.transpose(Vt), np.transpose(U)) # X should be orthogonal so, det(X) = 1 

    ################# Step 5) #################
    det_X = np.linalg.det(X)
    if det_X < 0:
        print("algorithm falied!")
        return 0

    Rot = copy.deepcopy(X)
    Trsl = target_mean - np.dot(Rot, source_mean.T)
    transformation = np.eye(4)
    transformation[:3, :3] = Rot
    transformation[:3 ,3] = Trsl.T

    return Rot, Trsl, transformation

###################### 1. downsample point cloud data ######################
############################################################################
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
#######################################################################
target_kdtree = o3d.geometry.KDTreeFlann(target_downsampled_voxel_indx)
num_of_selection = len(np.asarray(source_downsampled_voxel_indx.points)) # The number of neighbors to find

max_iteration = 4

for itr in np.arange(max_iteration):
# 'closest_pointset' below will have same address with the one inside of the method 'Match'
    closest_pointset = Match(
        source_downsampled_voxel_indx, target_kdtree, num_of_selection)
    line_set = Visualize(source_downsampled_voxel_indx, target_downsampled_voxel_indx,
            num_of_selection, closest_pointset)

    ###################### 3. align point cloud data #######################
    ######(for test reason, it doesn't include weighing and rejecting)######
    ########################################################################

    source_index = np.arange(0, num_of_selection)
    target_index = closest_pointset
    source_points, target_points = SelectPointSet(
        source_index, source_downsampled_voxel_indx, target_index, target_downsampled_voxel_indx)
    # The shape of source data and The shape of target data should be same
    print("The shape of source data: ", np.shape(source_points))
    print("The shape of target data: ", np.shape(target_points))

    Rt, Ts, transformation = Align(
        source_points, target_points, num_of_selection, closest_pointset)

    source_downsampled_voxel_indx.transform(transformation)
    o3d.visualization.draw_geometries([source_downsampled_voxel_indx, target_downsampled_voxel_indx, line_set])

    error_prev = (target_points - source_points)
    least_square_prev = np.mean(error_prev**2)

    prev_least_square = 0
    error = (target_points - np.transpose(np.dot(Rt, source_points.T)) + [Ts])
    least_square = np.mean(error**2)
    print(least_square)
    delta = np.abs(least_square - prev_least_square)
