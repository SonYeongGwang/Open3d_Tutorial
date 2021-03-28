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
            self.pcd = o3d.io.read_point_cloud(self.ModelPath)
            # self.pcd.scale(0.024, ([0, 0, 0]))
            self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn = 30))
            o3d.visualization.draw_geometries([self.pcd], point_show_normal=True)

            pts = np.asarray(self.pcd.points)
            norm = np.asarray(self.pcd.normals)
            self.model = np.concatenate((pts, norm), axis=1).astype('float32')
            self.scene = cv.ppf_match_3d.loadPLYSimple(self.ScenePath, SceneNor)

        elif self.fFormat == 'STL':
            print("Loading point cloud data...")
            # Load from STL using open3D API and extract points, normals
            mesh = o3d.io.read_triangle_mesh(self.ModelPath)
            self.pcd = mesh.sample_points_uniformly(number_of_points=4000)
            # self.pcd.scale(0.024, ([0, 0, 0]))
            self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn = 30))
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

    def Train(self, relativeSamplingStep=0.045, relativeDistanceStep=0.05):
        '''
        relativeSamplingStep = proportion to distance between points(0.025 - 0.05)
        relativeDistanceStep = inversely proportion to collision rate in hash table
                                (0.025-0.05 are sensible)
        '''
        self.detector = cv.ppf_match_3d_PPF3DDetector(relativeSamplingStep, relativeDistanceStep)
        print("training model...")
        self.detector.trainModel(self.model)

        print("complete training")

    def Match(self, relativeSceneSampleStep=1.0/2, relativeSceneDistance=0.05):
        '''
        relativeSceneSampleStep = proportion to sampling rate (1/a)->
        relativeSceneDistance = inversely proportion to collision rate in hash table
                                (0.025-0.05 are sensible)
        '''
        print("matching model to scene...")
        self.results = self.detector.match(self.scene, relativeSceneSampleStep, relativeSceneDistance)

        print("complete matching")
        return self.results

    def UpScale(self, HT):
        HT_in = copy.deepcopy(HT)
        HT_x = HT_in[:-1, 0]
        HT_y = HT_in[:-1, 1]
        HT_z = HT_in[:-1, 2]

        res_x = np.sqrt(HT_x[0]**2 + HT_x[1]**2 + HT_x[2]**2)
        res_y = np.sqrt(HT_y[0]**2 + HT_y[1]**2 + HT_y[2]**2)
        res_z = np.sqrt(HT_z[0]**2 + HT_z[1]**2 + HT_z[2]**2)

        scale_factor = np.mean((res_x, res_y, res_z))
        print("scale_factor:{}".format(np.mean((res_x, res_y, res_z))))

        HT_rot = HT_in[:3, :3]
        scale_factor = np.mean((res_x, res_y, res_z))
        HT_rot_upscaled = HT_rot / scale_factor

        HT_in[:3, :3] = HT_rot_upscaled

        return HT_in      

    def Visualize(self, results, line=True, box=True):
        flag = 0

        pose = results[0].pose
        pose = self.UpScale(pose)
        print("Scaled Pose: ")
        print(pose)

        org_color = [0.7, 0.2, 0.0]
        est_color = [1, 0.7, 0]

        if self.fFormat == 'PLY':
            model = o3d.io.read_point_cloud(self.ModelPath)
            # model.scale(0.024, ([0, 0, 0]))
        else:
            model = o3d.io.read_triangle_mesh(self.ModelPath)
            model = model.sample_points_uniformly(number_of_points=5000)
            
            # model.scale(0.024, ([0, 0, 0]))

        scene = o3d.io.read_point_cloud(self.ScenePath)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            model, scene, 0.02, pose,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))  #더 알아 볼 필요 있음

        new_pose = reg_p2p.transformation
        print("ICP Pose: ")
        print(new_pose)

        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        mesh_trans = copy.deepcopy(mesh)
        mesh_trans.transform(new_pose)

        model_estimate = copy.deepcopy(model)

        model.paint_uniform_color(org_color)
        model_estimate.paint_uniform_color(est_color)

        model_estimate_without_ICP = copy.deepcopy(model_estimate)

        model_estimate_without_ICP.transform(pose)
        model_estimate.transform(new_pose)
        
        # print("Scene")
        o3d.visualization.draw_geometries([scene, mesh, model, mesh_trans])

        if line:
            np.random.seed(7)
            indx = np.random.randint(0, len(model.points), (len(model.points)/2,))

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
                    [model, model_estimate, line_set, obb_org, obb_est, mesh_trans])
            
            else:
                o3d.visualization.draw_geometries(
                    [model, model_estimate_without_ICP, obb_org, obb_est, scene, mesh, mesh_trans])
                o3d.visualization.draw_geometries(
                    [model, model_estimate, obb_org, obb_est, scene, mesh, mesh_trans])
        
        else:
            if flag:
                o3d.visualization.draw_geometries(
                    [model, model_estimate, line_set])

            else:
                o3d.visualization.draw_geometries(
                    [model, model_estimate, scene])

model_path = '/home/a/mouse_data_set/mouse_data_main/RealSizeMouseModel.ply'
scene_path = '/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply'

# model_path = '/home/a/Open3d_Tutorial/Surface_matching/model2use.ply'
# scene_path = '/home/a/Open3d_Tutorial/Surface_matching/scene2use.ply'

# model_path = '/home/a/mouse_data_set/mouse_data_main/m185_2.stl'
# scene_path = '/home/a/mouse_data_set/mouse_data_scene/cropped/mouse_scene_crop.ply'

# scene_path = '/home/a/plz.ply'

sp = SurfaceMatching(fFormat='PLY', ModelPath=model_path, ScenePath=scene_path, ModelNor=1, SceneNor=1)
print(sp)
sp.Train()
results = sp.Match()
print(results[0].pose)
sp.Visualize(results, box=True, line=True)


####################################################################################
#                                    test zone                                     #
####################################################################################

# print("Loading point cloud data...")
# # Load from STL using open3D API and extract points, normals
# mesh_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
# mesh = o3d.io.read_triangle_mesh(model_path)
# pcd = mesh.sample_points_uniformly(number_of_points=5000)
# pcd_sce = o3d.io.read_point_cloud(scene_path)
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn = 30))
# # pcd.estimate_normals(
# #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
# o3d.visualization.draw_geometries([pcd, mesh_coor], point_show_normal=True)
# o3d.visualization.draw_geometries([mesh_coor, pcd])
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