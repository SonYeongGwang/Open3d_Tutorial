import numpy as np
import cv2 as cv
import open3d as o3d
import copy
import matplotlib.pyplot as plt

class SurfaceMatching:

    def __init__(self, ModelPath, ScenePath, ModelNor=0, SceneNor=0, radius=5.0, max_nn=40):
        self.ModelPath = ModelPath 
        self.ScenePath = ScenePath 
        self.ModelNor = ModelNor 
        self.SceneNor = SceneNor 
        self.radius = radius 
        self.max_nn = max_nn

        print("Loading point cloud data...")
        self.model = cv.ppf_match_3d.loadPLYSimple(self.ModelPath, ModelNor)
        self.scene = cv.ppf_match_3d.loadPLYSimple(self.ScenePath, SceneNor)

    def __str__(self):
        return "Model_matrix:{}, Scene_matrix:{}".format(np.shape(self.model), np.shape(self.scene))

    def Train(self, relativeSamplingStep=0.04, relativeDistanceStep=0.05):
        '''
        relativeSamplingStep = proportion to distance between points(0.025 - 0.05)
        relativeDistanceStep = inversely proportion to collision rate in hash table
                                (0.025-0.05 are sensible)
        '''
        self.detector = cv.ppf_match_3d_PPF3DDetector(relativeSamplingStep, relativeDistanceStep)
        print("training model...")
        self.detector.trainModel(self.model)

        print("complete training")

    def Match(self, relativeSceneSampleStep=1.0/40.0, relativeSceneDistance=0.05):
        '''
        relativeSceneSampleStep = need to be defined***
        relativeSceneDistance = inversely proportion to collision rate in hash table
                                (0.025-0.05 are sensible)
        '''
        print("matching model to scene...")
        self.results = self.detector.match(self.scene, relativeSceneSampleStep, relativeSceneDistance)

        print("complete matching")
        return self.results

    def Visualize(self, results, line=True, box=True):
        flag = 0
        pose = results[0].pose

        org_color = [0.7, 0.2, 0.0]
        est_color = [1, 0.7, 0]

        model = o3d.io.read_point_cloud(self.ModelPath)
        scene = o3d.io.read_point_cloud(self.ScenePath)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)

        model_estimate = copy.deepcopy(model)

        model.paint_uniform_color(org_color)
        model_estimate.paint_uniform_color(est_color)

        model_estimate.transform(pose)
        
        # print("Scene")
        # o3d.visualization.draw_geometries([scene, mesh])

        if line:
            np.random.seed(7)
            indx = np.random.randint(0, len(model.points), (20,))

            sel_point_org = model.select_by_index(indx)
            sel_point_org = np.asarray(sel_point_org.points)
            sel_point_est = model_estimate.select_by_index(indx)
            sel_point_est = np.asarray(sel_point_est.points)

            points = np.concatenate((sel_point_org, sel_point_est), axis=0)

            lines = np.asarray([[i, i + len(points)/2]for i in np.arange(len(points)/2)])
            lines.astype('int32')
            colors = [[1, 0, 0] for i in range(len(lines))]
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            flag = flag + 1

        if box:
            obb_org = model.get_oriented_bounding_box()
            obb_org.color = org_color
            obb_est = model_estimate.get_oriented_bounding_box()
            obb_est.color = est_color

            if flag:
                o3d.visualization.draw_geometries(
                    [model, model_estimate, line_set, obb_org, obb_est])
            
            else:
                o3d.visualization.draw_geometries(
                    [model, model_estimate, obb_org, obb_est, scene, mesh])
        
        else:
            if flag:
                o3d.visualization.draw_geometries(
                    [model, model_estimate, line_set])

            else:
                o3d.visualization.draw_geometries(
                    [model, model_estimate, scene, mesh])


sp = SurfaceMatching('/home/a/Open3d_Tutorial/cv/model2use.ply', '/home/a/Open3d_Tutorial/cv/scene2use.ply', ModelNor=1, SceneNor=1)
print(sp)
sp.Train()
results = sp.Match()
print(results[0].pose)
sp.Visualize(results, box=False, line=False)


'''####################################################################################
test zone
'''####################################################################################

# print("Model")

# transformation = np.asarray([[ 9.85484315e-01, -1.69657577e-01,  6.08036725e-03, -1.08048926e+02],
#  [ 1.24772814e-01,  6.99547004e-01, -7.03609078e-01, -5.39479328e+02],
#  [ 1.15119109e-01,  6.94154375e-01,  7.10561254e-01, -2.13442662e+02],
#  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)
# mesh_trans = copy.deepcopy(mesh)
# mesh_trans.transform(transformation)
# model = o3d.io.read_point_cloud('./parasaurolophus_6700.ply')
# model.translate([0, 100, 600])
# scene = o3d.io.read_point_cloud('./rs1_normals.ply')
# scene.translate([0, 0, 600])
# print(model.get_center())
# print(scene.get_center())
# o3d.visualization.draw_geometries([model, mesh, mesh_trans, scene])
# o3d.visualization.draw_geometries([scene, mesh])

# o3d.visualization.draw_geometries([scene, mesh])
# o3d.io.write_point_cloud('model2use.ply', model, write_ascii=True, print_progress=True)


# model = o3d.io.read_point_cloud('/home/a/Open3d_Tutorial/cv/model2use.ply')
# scene = o3d.io.read_point_cloud('/home/a/Open3d_Tutorial/cv/scene2use.ply')
# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)
# o3d.visualization.draw_geometries([model, mesh])
# o3d.visualization.draw_geometries([scene, mesh])

'''####################################################################################
test zone
'''####################################################################################