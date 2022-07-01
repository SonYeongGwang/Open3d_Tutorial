import numpy as np
import open3d as o3d

gripper = o3d.io.read_triangle_mesh("/home/a/Desktop/gripper22.stl")
gripper.compute_vertex_normals()

depth_start_point_from_glass = 0.0042 #m
end_to_cam_glass = 0.057
cam_center2rgb = 0.0325
end_to_cam_height = 0.0575
end_to_cup_length = 0.2835

end = o3d.geometry.TriangleMesh.create_coordinate_frame(0.04)
eye = o3d.geometry.TriangleMesh.create_coordinate_frame(0.02)
cup = o3d.geometry.TriangleMesh.create_coordinate_frame(0.04)

end2cam = np.array([[ 1,    0,    0,   -cam_center2rgb],
                    [ 0,    1,    0,   -end_to_cam_height],
                    [ 0,    0,    1,    end_to_cam_glass - depth_start_point_from_glass],
                    [ 0,    0,    0,    1]])

end2tool = np.array([[1,    0,    0,    0],
                    [ 0,    1,    0,    0],
                    [ 0,    0,    1,    end_to_cup_length],
                    [ 0,    0,    0,    1]])

cam2end = np.linalg.inv(end2cam)
print("cam2end:\n",cam2end)
print()

print("end2cam:\n",end2cam)
print()

cam2tool = np.dot(cam2end, end2tool)
print("cam2tool:\n",cam2tool)

eye.transform(end2cam)
cup.transform(end2tool)

# o3d.visualization.draw_geometries([gripper, end, eye, cup])