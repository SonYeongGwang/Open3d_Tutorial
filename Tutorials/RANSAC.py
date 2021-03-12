import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

pcd = o3d.io.read_point_cloud("/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply")
plane_model, inliers = pcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)
[a, b, c, d] = plane_model
print("Plane equation: {:.2f}x + {:.2f}y + {:.2f}z + {:.2f} = 0".format(a, b, c, d))

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0, 0, 0.8])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([inlier_cloud])
o3d.visualization.draw_geometries([outlier_cloud])

# o3d.io.write_point_cloud('/home/a/mouse_data_set/mouse_data_scene/mouse_scene_randac.ply', outlier_cloud, write_ascii=True)
# X = np.asarray(pcd.points)[:,0]
# Y = np.asarray(pcd.points)[:,1]
# Z = np.asarray(pcd.points)[:,2]
# print("Xmax:{}, Xmin:{}, Ymax:{}, Ymin:{}".format(np.argmax(X), np.argmin(X), np.argmax(Y), np.argmin(Y)))
# x = np.linspace(3000, 70000, 100)
# y = np.linspace(10, 120000, 100)
# x, y = np.meshgrid(x, y)
# z = -1/c * (a*x + b*y + d)

# ax = plt.axes(projection='3d')
# ax.contour3D(x, y, z, 100, cmap=plt.cm.rainbow)
# plt.show()