import numpy as np
import cv2 as cv
import open3d as o3d
import copy
import matplotlib.pyplot as plt



class SurfaceMatching:

    def __init__(self, fFormat, ModelPath, ScenePath, ModelNor=0, SceneNor=0, radius=5.0, max_nn=40):
        self.ModelPath = ModelPath 
        self.ScenePath = ScenePath 
        self.ModelNor = ModelNor 
        self.SceneNor = SceneNor 
        self.radius = radius 
        self.max_nn = max_nn
        self.fFormat = fFormat

        if self.fFormat == 'PLY':
            print("Loading point cloud data...")
            self.model = cv.ppf_match_3d.loadPLYSimple(self.ModelPath, ModelNor)
            self.scene = cv.ppf_match_3d.loadPLYSimple(self.ScenePath, SceneNor)

        elif self.fFormat == 'STL':
            print("Loading point cloud data...")
            # Load from STL using open3D API and extract points, normals
            mesh = o3d.io.read_triangle_mesh(self.ModelPath)
            self.pcd = mesh.sample_points_uniformly(number_of_points=2000)
            self.pcd.scale(0.0265, ([0, 0, 0]))
            self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
            o3d.visualization.draw_geometries([self.pcd], point_show_normal=True)

            pts = np.asarray(self.pcd.points)
            norm = np.asarray(self.pcd.normals)
            self.model = np.concatenate((pts, norm), axis=1).astype('float32')

            # Load directly from PLY file
            self.scene = cv.ppf_match_3d.loadPLYSimple(self.ScenePath, SceneNor)

        else:
            print("Error: File type should be .PLY or .STL")

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

        if self.fFormat == 'PLY':
            model = o3d.io.read_point_cloud(self.ModelPath)
        else:
            model = o3d.io.read_triangle_mesh(self.ModelPath)
            model = model.sample_points_uniformly(number_of_points=1500)
            model.scale(0.0265, ([0, 0, 0]))

        scene = o3d.io.read_point_cloud(self.ScenePath)

        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

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

model_path = '/home/a/mouse_data_set/mouse_data_main/m185_2.stl'
scene_path = '/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply'
# model_path = '/home/a/Open3d_Tutorial/Surface_matching/model2use.ply'
# scene_path = 'Surface_matching/scene2use.ply'
sp = SurfaceMatching(fFormat='STL', ModelPath=model_path, ScenePath=scene_path, ModelNor=1, SceneNor=1)
print(sp)
sp.Train()
results = sp.Match()
print(results[0].pose)
sp.Visualize(results, box=True, line=False)


####################################################################################
#                                    test zone                                     #
####################################################################################

# print("Loading point cloud data...")
# # Load from STL using open3D API and extract points, normals
# mesh_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
# mesh = o3d.io.read_triangle_mesh(model_path)
# pcd = mesh.sample_points_uniformly(number_of_points=5000)
# pcd_scale = copy.deepcopy(pcd)
# pcd_scale.scale(0.0265, ([0, 0, 0]))
# pcd_sce = o3d.io.read_point_cloud(scene_path)
# # pcd.estimate_normals(
# #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
# # o3d.visualization.draw_geometries([pcd], point_show_normal=True)
# o3d.visualization.draw_geometries([mesh_coor, pcd_scale, pcd_sce])
# print(pcd.get_min_bound())
# print(pcd.get_max_bound())

# pts = np.asarray(pcd.points)
# norm = np.asarray(pcd.normals)
# model = np.concatenate((pts, norm), axis=1).astype('float32')
# print(type(model[0][0]))

# scene = cv.ppf_match_3d.loadPLYSimple(scene_path, 1)
# print(type(scene[0][0]))

####################################################################################
#                                   test zone                                      #
####################################################################################

# model = o3d.io.read_point_cloud('/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply')
# o3d.visualization.draw_geometries([model], point_show_normal=True)