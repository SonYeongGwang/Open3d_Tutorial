import numpy as np
import cv2 as cv
import open3d as o3d
import copy
import matplotlib.pyplot as plt


model = o3d.io.read_point_cloud('cv/parasaurolophus_6700.ply')
scene = o3d.io.read_point_cloud('cv/rs1_normals.ply')
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)
model_dw = model.voxel_down_sample(voxel_size=0.005)
model_nor = copy.deepcopy(model)
model_nor.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5.0, max_nn=40))

o3d.visualization.draw_geometries([model, mesh])
# o3d.visualization.draw_geometries([model, mesh])
# o3d.visualization.draw_geometries([model_nor], point_show_normal=True)
o3d.visualization.draw_geometries([scene, mesh])


pcd_xyz = np.asarray(model.points)
pcd_nor = np.asarray(model_nor.normals)
pcd = np.concatenate((pcd_xyz, pcd_nor), axis=1)
pcd = np.array(pcd, dtype='float32')
print("pcd_xyz: ", np.shape(pcd_xyz), " pcd_nor: ", np.shape(pcd_nor), " pcd: ", np.shape(pcd))

'''
### scatter using matplotlib ###
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.scatter(pcd[:, 0], pcd[:, 1], pcd[:, 2])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
'''


modelname = "parasaurolophus_6700"
N = 2

detector = cv.ppf_match_3d_PPF3DDetector(relativeSamplingStep=0.025, relativeDistanceStep=0.05)

test = cv.ppf_match_3d.loadPLYSimple('cv/parasaurolophus_6700.ply', 1)
test_scene = cv.ppf_match_3d.loadPLYSimple('cv/rs1_normals.ply', 1)
# print(pcd[:, 5])
# print(test[:, 5])

# trainModel(float32 matrix(None, 6))

detector.trainModel(test)

results = detector.match(test_scene, relativeSceneSampleStep=1.0/40.0, 	relativeSceneDistance=0.05)

icp = cv.ppf_match_3d_ICP(100)
_, results = icp.registerModelToScene(test, test_scene, results[:N])


# help(cv.ppf_match_3d_Pose3D)

print("Poses: ")
for i, result in enumerate(results):
    #result.printPose()
    print("\n-- Pose to Model Index %d: NumVotes = %d, Residual = %f\n%s\n" % (result.modelIndex, result.numVotes, result.residual, result.pose))
    if i == 0:
        pct = cv.ppf_match_3d.transformPCPose(test, result.pose)
        # cv.ppf_match_3d.writePLY(pct, "%sPCTrans.ply" % modelname)


print(results[0].pose)

print("Translate model to match the scene")
model.transform(results[0].pose)
model.paint_uniform_color([1, 0.7, 0])
o3d.visualization.draw_geometries([model, scene])