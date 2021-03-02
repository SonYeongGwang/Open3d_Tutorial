import numpy as np
import open3d as o3d
import copy

# copy ply data and paint colors
pcd = o3d.io.read_point_cloud('./eagle.ply')
pcd = pcd.voxel_down_sample(voxel_size=0.09)
pcd.paint_uniform_color([1, 0.0, 0.0])
pcd_rot = copy.deepcopy(pcd)
pcd_rot.paint_uniform_color([0.0, 0.0, 1])

# get center
center = pcd.get_center()

# make a coordinate mesh
coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
coor_cen = copy.deepcopy(coor)
coor_cen.translate(center)
coor_cen_rot = copy.deepcopy(coor_cen)

# define rotate value
R = coor.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))

# rotate coor & pcd & display the result
coor_cen_rot.rotate(R, center=center)
pcd_rot.rotate(R, center=center)
o3d.visualization.draw_geometries([pcd, coor, coor_cen, coor_cen_rot, pcd_rot])

# translate pcd and coor and display the result
pcd_scale = copy.deepcopy(pcd).translate((4, 0, 0))
pcd_scale2 = copy.deepcopy(pcd).translate((-4, 0, 0))
pcd_scale.scale(0.5, center=pcd_scale.get_center())
pcd_scale2.scale(0.5, center=(0, 0, 0))
o3d.visualization.draw_geometries([pcd, pcd_scale, pcd_scale2, coor])