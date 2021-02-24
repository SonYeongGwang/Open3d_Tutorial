import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("./fragment.ply")
vol = o3d.visualization.read_selection_polygon_volume("./Crop/cropped.json")

chair = vol.crop_point_cloud(pcd)

dists = pcd.compute_point_cloud_distance(chair)
# source=pcd & target=chair
# It computes (for each point in the source point cloud) the distance to the closest point in the target point cloud
dists = np.asarray(dists)
ind = np.where(dists > 0.20)[0]
print(ind)
pcd_without_chair = pcd.select_by_index(ind)
o3d.visualization.draw_geometries([pcd_without_chair])