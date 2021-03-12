import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("/home/a/mouse_data_set/mouse_data_scene/closed.ply")
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh_z = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh_z = mesh_z.translate((0, -0.15, -0.8))
# print("Global Center: {}".format(mesh.get_center()))
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd, mesh_z, mesh])
print(pcd.points[0])

print("Load a polygon volume and use it to crop the original point cloud")
vol = o3d.visualization.read_selection_polygon_volume("./Crop/cropped_mouse_loose.json")
box = vol.crop_point_cloud(pcd)
print(box.get_center())  # center of the geometry coordinates
o3d.visualization.draw_geometries([box])

aabb = box.get_axis_aligned_bounding_box()
aabb.color = (0, 0, 1)
o3d.visualization.draw_geometries([box, aabb])

# plane_model, inliers = box.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=100)
# [a, b, c, d] = plane_model
# print("Plane equation: {:.2f}x + {:.2f}y + {:.2f}z + {:.2f} = 0".format(a, b, c, d))

# inliers_cloud = box.select_by_index(inliers)
# inliers_cloud.paint_uniform_color([0.8, 0, 0])
# o3d.visualization.draw_geometries([pcd, inliers_cloud])

print("Recompute the normal of the downsampled point cloud")
box.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([box], point_show_normal=True)

# o3d.io.write_point_cloud('/home/a/mouse_data_set/mouse_data_scene/mouse_scene_crop.ply', box, write_ascii=True)