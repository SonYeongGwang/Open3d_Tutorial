import numpy as np
from InteractiveClosestPoint import *
import open3d as o3d
import copy

actual_transforamtion = np.array([[ 9.23879527e-01,  3.15662565e-09,  3.82683445e-01,  7.56464850e-04],
 [-6.04162158e-09,  1.00000000e+00,  6.33710401e-09,  1.23219635e-11],
 [-3.82683445e-01, -8.16674915e-09,  9.23879527e-01, -1.67510920e-04],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# actual_transforamtion = np.array([[-0.98092033, -0.06828617, -0.18202281, -0.00162741],
#  [-0.1888786,   0.5564983,   0.80909488, -0.0411363 ],
#  [ 0.04604539,  0.82803783, -0.55877829, -0.5873832 ],
#  [ 0.,          0.,          0.,          1.        ]])

# trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
#                         [-0.139, 0.967, -0.215, 0.7],
#                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])

org_color = [0.7, 0.2, 0.0]
est_color = [1, 0.7, 0]

# model_path = '/home/a/mouse_data_set/mouse_data_main/RealSizeMouseModel.ply'
# scene_path = '/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply'

# model_path = '/home/a/Open3d_Tutorial/ICP/cloud_bin_0.pcd'
# scene_path = '/home/a/Open3d_Tutorial/ICP/cloud_bin_1.pcd'

print("Loading PCD...")
model_path = '/home/a/mouse_data_set/mouse_data_main/mouse_model_ransac.ply'
scene_path = '/home/a/mouse_data_set/mouse_data_scene/senthetic/mouse.ply'

model = o3d.io.read_point_cloud(model_path)
model.paint_uniform_color(org_color)
scene = o3d.io.read_point_cloud(scene_path)
scene.paint_uniform_color(est_color)
# model.transform(transforamtion)
model_refine = copy.deepcopy(model)

o3d.visualization.draw_geometries([model, scene])

print("downsample larger pcd")
model_points = len(np.asarray(model.points))
scene_points = len(np.asarray(scene.points))
target_sample = scene_points if model_points > scene_points else model_points
print("target_sample_len:", target_sample)
indx = np.random.choice(np.arange(0, model_points), size=(target_sample, ), replace=False)
# print(np.shape(indx))

model_downsampled = model.select_by_index(indx)
print(model_downsampled)

o3d.visualization.draw_geometries([model_downsampled, scene])

model_pcd = np.asarray(model_downsampled.points)
scene_pcd = np.asarray(scene.points)

print("Implementing ICP")

T, distance, i = icp(model_pcd, scene_pcd, max_iterations=100, tolerance=0.00005)
print(T)

model_refine.transform(T)
o3d.visualization.draw_geometries([model_refine, scene])

# it is important to keep track of local minimum