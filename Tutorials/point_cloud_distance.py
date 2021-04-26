import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("Tutorials/fragment.ply")
vol = o3d.visualization.read_selection_polygon_volume("./Crop/cropped.json")
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(1)
chair = vol.crop_point_cloud(pcd)

dists = pcd.compute_point_cloud_distance(chair)
# source=pcd & target=chair
# It computes (for each point in the source point cloud) the distance to the closest point in the target point cloud
dists = np.asarray(dists)
ind = np.where(dists > 0.01)[0]
print(ind)
pcd_without_chair = pcd.select_by_index(ind)
o3d.visualization.draw_geometries([pcd_without_chair, mesh])

'''
bounding volumes
'''
# https://hoodymong.tistory.com/8
# obb 와 aabb의 차이점
aabb = chair.get_axis_aligned_bounding_box()
aabb.color = (0, 0, 1)
obb = chair.get_oriented_bounding_box()
obb.color = (1, 0, 0)
o3d.visualization.draw_geometries([chair, aabb, obb, mesh])

'''
Convex hull(using Quick hull)
'''
downchair = chair.voxel_down_sample(voxel_size=0.05)
hull, _ = downchair.compute_convex_hull()
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((0, 0, 1))
o3d.visualization.draw_geometries([chair, hull_ls, mesh])