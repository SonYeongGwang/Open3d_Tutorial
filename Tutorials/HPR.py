import numpy as np
import open3d as o3d

camera_position = [0, 0.1, 0.15]
pcd = o3d.io.read_point_cloud("./Tutorials/Bunny.ply")
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1, [0, 0, 0])
camera = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1, camera_position)

o3d.visualization.draw_geometries([pcd, origin, camera])
radius = 100
_, pt_map = pcd.hidden_point_removal(camera_position, radius)
pcd = pcd.select_by_index(pt_map)
o3d.visualization.draw_geometries([pcd, origin, camera])
_, idx = pcd.remove_radius_outlier(80, 0.02)    # --> argument should be determined automatically
pcd = pcd.select_by_index(idx)
o3d.visualization.draw_geometries([pcd, origin, camera])