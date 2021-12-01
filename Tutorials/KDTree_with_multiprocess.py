import open3d as o3d
import numpy as np
import time
from multiprocessing import Pool


# t1 = time.time()
# pcd = o3d.io.read_point_cloud('Tutorials/eagle.ply')
# pcd_tree = o3d.geometry.KDTreeFlann(pcd)

# print(pcd)
# for i in range(len(pcd.points)):
#     [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], 100)
# print("Total execution: {}".format(time.time()-t1))

def search(idx):
    global pcd_tree
    [k, idxs, _] = pcd_tree.search_knn_vector_3d(pcd.points[idx], 100)
    print(k)
    return np.asarray(idxs)

t1 = time.time()
pcd = o3d.io.read_point_cloud('Tutorials/eagle.ply')
point_indies = np.arange(len(pcd.points))
pcd_tree = o3d.geometry.KDTreeFlann(pcd)
pool = Pool(processes=None)
pool.map(search, point_indies)
print("Total execution: {}".format(time.time()-t1))