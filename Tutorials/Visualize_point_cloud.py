import numpy as np
import open3d as o3d

'''
open3d uses 'm' for length unit 
'''

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("/home/a/mouse_data_set/mouse_data_main/mouse_model_randac.ply")
print(pcd)
print(np.asarray(pcd.points))
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
print("Global Center: {}".format(mesh.get_center()))
o3d.visualization.draw_geometries([pcd, mesh])

print("Downsample the point cloud with a voxel of 0.05")
downpcd = pcd.voxel_down_sample(voxel_size=0.09)
o3d.visualization.draw_geometries([downpcd])

print("Recompute the normal of the downsampled point cloud")
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([pcd], point_show_normal=True)
'''
The covariance analysis algorithm produces two opposite directions as normal candidates. 
Without knowing the global structure of the geometry, both can be correct. 
This is known as the normal orientation problem. 
Open3D tries to orient the normal to align with the original normal if it exists.
Otherwise, Open3D does a random guess. 
Further orientation functions such as orient_normals_to_align_with_direction and orient_normals_towards_camera_location need to be called if the orientation is a concern.
'''

# help(downpcd)

print("Print the normal vectors of the first 10 points")
print(np.asarray(downpcd.normals)[:10,:])

print("Load a polygon volume and use it to crop the original point cloud")
vol = o3d.visualization.read_selection_polygon_volume("./Crop/cropped.json")
chair = vol.crop_point_cloud(pcd)
o3d.visualization.draw_geometries([chair])