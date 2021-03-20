import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import open3d as o3d
import copy

# bounding_polygon = np.array([
# 		[ -0.1, -0.035, 0.0 ],
# 		[ 0.1, -0.035, 0.0 ],
# 		[ 0.1, 0.155, 0.0 ],
# 		[ -0.1, 0.155, 0.0 ],
#         [ -0.1, -0.035, 0.0 ]
# 	])

# bounding_polygon = bounding_polygon[:, [0, 1]]
# print(bounding_polygon)

# plt.figure()
# plt.plot(bounding_polygon[:,0], bounding_polygon[:,1])
# plt.show()

'''
model_path = '/home/a/realsense_box.STL'

mesh = o3d.io.read_triangle_mesh(model_path)
pcd = mesh.sample_points_uniformly(number_of_points=5000)
mesh_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
mesh_coor_center = copy.deepcopy(mesh_coor)
mesh_coor_center.translate([0, 0, (pcd.get_center())[2]])
pcd.translate([0, 0, -(pcd.get_center())[2]])
o3d.visualization.draw_geometries([pcd, mesh_coor, mesh_coor_center])
'''


'''
scene_path = '/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply'
model_path = '/home/a/mouse_data_set/mouse_data_main/mouse_CAD_Model.ply'

mesh_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
mesh_coor_trs = copy.deepcopy(mesh_coor)
mesh_coor_trs.translate([0, -0.018, -0.58])
mesh_coor_rot = copy.deepcopy(mesh_coor_trs)
R = mesh_coor_rot.get_rotation_matrix_from_xyz((np.pi / 4, 0, 0))
mesh_coor_rot.rotate(R, center=mesh_coor_rot.get_center())

model = o3d.io.read_point_cloud(model_path)
R = mesh_coor.get_rotation_matrix_from_xyz((0, np.pi*3.7/4, 0))
model.scale(0.024, ([0, 0, 0]))
model.rotate(R, mesh_coor.get_center())
model.translate([0, -0.025, 0])

scene = o3d.io.read_point_cloud(scene_path)
o3d.visualization.draw_geometries([scene])#, mesh_coor, mesh_coor_trs, mesh_coor_rot])
print(R)
scene.translate([0, 0.015, 0.58])
R = mesh_coor_rot.get_rotation_matrix_from_xyz((-np.pi / 4, 0, 0))
scene.rotate(R, mesh_coor.get_center())
o3d.visualization.draw_geometries([scene, mesh_coor, model])
'''



# model_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5.0, max_nn=50))
# o3d.visualization.draw_geometries([model_o3d], point_show_normal=True)


# test_scene = o3d.io.read_point_cloud('/home/a/mouse_data_set/mouse_data_scene/closed.ply')
# o3d.visualization.draw_geometries([test_scene])
# print(test_scene)


# o3d.io.write_point_cloud('closed.ply', test_scene, write_ascii=True)

'''
mouse_path = '/home/a/mouse_data_set/mouse_data_main/mouse_CAD_Model.ply'

mouse1 = o3d.io.read_point_cloud(mouse_path)
mouse1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn = 30))

mouse2 = o3d.io.read_point_cloud(mouse_path)
mouse2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.07, max_nn=30))

o3d.visualization.draw_geometries([mouse1], point_show_normal=True)
o3d.visualization.draw_geometries([mouse2], point_show_normal=True)
'''
# o3d.io.write_point_cloud('/home/a/Open3d_Tutorial/Surface_matching/scene2test.ply', scene, write_ascii=True)


HT = np.array([[ 2.90040405e-03,  5.49825709e-02, -3.71633300e-01, -6.06459842e-04],
 [-2.97180570e-01,  2.27689755e-01,  3.13670072e-02, -4.54455377e-02],
 [ 2.29821890e-01,  2.93729633e-01,  4.52504841e-02, -5.97049404e-01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

HT_x = HT[:-1, 0]
HT_y = HT[:-1, 1]
HT_z = HT[:-1, 2]

# print(np.cross(HT_x, HT_y))

res_x = np.sqrt(HT_x[0]**2 + HT_x[1]**2 + HT_x[2]**2)
res_y = np.sqrt(HT_y[0]**2 + HT_y[1]**2 + HT_y[2]**2)
res_z = np.sqrt(HT_z[0]**2 + HT_z[1]**2 + HT_z[2]**2)
print("x_element:", res_x)
print("y_element:", res_y)
print("z_element:", res_z)


# rescaling function
HT_rot = HT[:3, :3]
scale_factor = np.mean((res_x, res_y, res_z))
HT_rot_upscaled = HT_rot / scale_factor
print(HT_rot_upscaled)

HT[:3, :3] = HT_rot_upscaled
print(HT)

# def UpScale(HT):
#     HT_in = HT
#     HT_x = HT_in[:-1, 0]
#     HT_y = HT_in[:-1, 1]
#     HT_z = HT_in[:-1, 2]

#     res_x = np.sqrt(HT_x[0]**2 + HT_x[1]**2 + HT_x[2]**2)
#     res_y = np.sqrt(HT_y[0]**2 + HT_y[1]**2 + HT_y[2]**2)
#     res_z = np.sqrt(HT_z[0]**2 + HT_z[1]**2 + HT_z[2]**2)

#     scale_factor = np.mean((res_x, res_y, res_z))
#     print("scale_factor:{}".format(np.mean((res_x, res_y, res_z))))

#     HT_rot = HT_in[:3, :3]
#     scale_factor = np.mean((res_x, res_y, res_z))
#     HT_rot_upscaled = HT_rot / scale_factor

#     HT_in[:3, :3] = HT_rot_upscaled

#     return HT_in