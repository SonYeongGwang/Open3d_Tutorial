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

    def Visualize(self, results):
        pose = results[0].pose

        model = o3d.io.read_point_cloud(self.ModelPath)
        scene = o3d.io.read_point_cloud(self.ScenePath)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)

        model_estimate = copy.deepcopy(model)

        model.paint_uniform_color([0, 1, 0.7])
        model_estimate.paint_uniform_color([1, 0.7, 0])

        model_estimate.transform(pose)
        # print("Model")
        # o3d.visualization.draw_geometries([model, mesh])
        # print("Scene")
        # o3d.visualization.draw_geometries([scene, mesh])
        o3d.visualization.draw_geometries([model, scene, model_estimate, mesh])

sp = SurfaceMatching('./parasaurolophus_6700.ply', './rs1_normals.ply', ModelNor=1, SceneNor=1)
print(sp)
sp.Train()
results = sp.Match()
print(results[0].pose)
sp.Visualize(results)