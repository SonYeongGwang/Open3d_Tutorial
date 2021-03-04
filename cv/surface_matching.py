import numpy as np
import cv2 as cv
import open3d as o3d
import copy
import matplotlib.pyplot as plt

class SurfaceMatching:

    def __init__(self, ModelPath, ScenePath, ModelNor, SceneNor, radius, max_nn):
        self.ModelPath = ModelPath 
        self.ScenePath = ScenePath 
        self.ModelNor = ModelNor 
        self.SceneNor = SceneNor 
        self.radius = radius 
        self.max_nn = max_nn

        print("Loading point cloud data...")
        self.model = cv.ppf_match_3d.loadPLYSimple(self.ModelPath)
        self.scene = cv.ppf_match_3d.loadPLYSimple(self.ScenePath)

    def Train(self, relativeSamplingStep=0.025, relativeDistanceStep=0.05):
        self.detector = cv.ppf_match_3d_PPF3DDetector(relativeSamplingStep, relativeDistanceStep)
        print("training model...")
        self.detector.trainModel(self.model)