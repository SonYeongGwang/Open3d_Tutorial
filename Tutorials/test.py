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
model_path = '/home/a/mouse_data_set/mouse_data_main/m185_2.stl'

mesh = o3d.io.read_triangle_mesh(model_path)
pcd = mesh.sample_points_uniformly(number_of_points=5000)
mesh_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
mesh_coor_center = copy.deepcopy(mesh_coor)
mesh_coor_center.translate([0, 0, (pcd.get_center())[2]])
pcd.translate([0, 0, -(pcd.get_center())[2]])
o3d.visualization.draw_geometries([pcd, mesh_coor, mesh_coor_center])
'''

scene_path = '/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply'
model_path = '/home/a/mouse_data_set/mouse_data_main/mouse_CAD_Model.ply'

mesh_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
mesh_coor_trs = copy.deepcopy(mesh_coor)
mesh_coor_trs.translate([0, -0.17, -0.97])
mesh_coor_rot = copy.deepcopy(mesh_coor_trs)
R = mesh_coor_rot.get_rotation_matrix_from_xyz((np.pi / 6, 0, 0))
mesh_coor_rot.rotate(R, center=mesh_coor_rot.get_center())

model = o3d.io.read_point_cloud(model_path)
R = mesh_coor.get_rotation_matrix_from_xyz((0, np.pi*3.4/4, 0))
model.scale(0.024, ([0, 0, 0]))
model.rotate(R, mesh_coor.get_center())

scene = o3d.io.read_point_cloud(scene_path)
o3d.visualization.draw_geometries([scene, mesh_coor, mesh_coor_trs, mesh_coor_rot])
print(R)
scene.translate([0, 0.17, 0.97])
R = mesh_coor_rot.get_rotation_matrix_from_xyz((-np.pi / 6, 0, 0))
scene.rotate(R, mesh_coor.get_center())
o3d.visualization.draw_geometries([scene, mesh_coor, model])
# model_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5.0, max_nn=50))
# o3d.visualization.draw_geometries([model_o3d], point_show_normal=True)


# o3d.io.write_point_cloud('mouse_CAD_Model.ply', pcd, write_ascii=True)