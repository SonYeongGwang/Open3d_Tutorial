import numpy as np
import open3d as o3d
import math
import copy

mesh_org = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
# o3d.visualization.draw_geometries([mesh_org])

roll = 30

roll = roll * math.pi/180.0

transformation1 = np.array([[1, 0, 0, 0],
                           [0, math.cos(roll), -math.sin(roll), 0],
                           [0, math.sin(roll), math.cos(roll), 0],
                           [0, 0, 0, 1]])

transformation2 = np.array([[1, 0, 0, 2],
                            [0, 1, 0, 0],
                            [0, 0, 1, 2],
                            [0, 0, 0, 10]])

transformation3 = np.array([
    [ 2.90040405e-03,  5.49825709e-02, -3.71633300e-01, -6.06459842e-04],
    [-2.97180570e-01,  2.27689755e-01,  3.13670072e-02, -4.54455377e-02],
    [ 2.29821890e-01,  2.93729633e-01,  4.52504841e-02, -5.97049404e-01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
     ])


transformation3_2 = np.array([[ 2.90040405e-03,  5.49825709e-02, -3.71633300e-01, -6.06459842e-04],
 [-2.97180570e-01,  2.27689755e-01,  3.13670072e-02, -4.54455377e-02],
 [ 2.29821890e-01,  2.93729633e-01,  4.52504841e-02, -5.97049404e-01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

def UpScale(HT):
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

transformation3_2_processed = UpScale(transformation3_2)
# print(transformation3_2_processed)

# print(transformation_transl[:,3])
# print(transformation3_2[:, 3])

mesh_rot_roll = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2)
mesh_trans = o3d.geometry.TriangleMesh.create_coordinate_frame(size=7)
mesh_trans2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=6)
mesh_transl = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2)

mesh_rot_roll.transform(transformation1)
# mesh_trans.transform(transformation2)
mesh_trans2.transform(transformation3_2_processed)

o3d.visualization.draw_geometries([mesh_trans,  mesh_rot_roll, mesh_trans2])