import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("./test_data2.ply")
# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
# mesh_z = o3d.geometry.TriangleMesh.create_coordinate_frame()
# mesh_z = mesh_z.translate((0, 0, -0.50))
# print("Global Center: {}".format(mesh.get_center()))
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

print("Load a polygon volume and use it to crop the original point cloud")
vol = o3d.visualization.read_selection_polygon_volume("./Crop/cropped_box.json")
box = vol.crop_point_cloud(pcd)
o3d.visualization.draw_geometries([box])

aabb = box.get_axis_aligned_bounding_box()
aabb.color = (0, 0, 1)
o3d.visualization.draw_geometries([box, aabb])