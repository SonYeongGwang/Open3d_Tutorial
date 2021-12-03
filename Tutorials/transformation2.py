import numpy as np
import open3d as o3d

end = o3d.geometry.TriangleMesh.create_coordinate_frame(0.04)
cam = o3d.geometry.TriangleMesh.create_coordinate_frame(0.02)
eye = o3d.geometry.TriangleMesh.create_coordinate_frame(0.01)
cup = o3d.geometry.TriangleMesh.create_coordinate_frame(0.02)
end2cam = np.array([[np.math.cos(np.math.pi/4),  -np.math.sin(np.math.pi/4), 0, 0.0312*np.math.cos(np.math.pi/4)],
               [ np.math.sin(np.math.pi/4),   np.math.cos(np.math.pi/4),    0,    -0.0312*np.math.cos(np.math.pi/4)],
               [ 0,                            0,               1,                0.039],
               [ 0,                  0,  0,                         1]])
cam2eye = np.array([[1,  0, 0, -0.0325],
                    [ 0, 1, 0, 0],
                    [ 0, 0, 1, 0],
                    [ 0, 0, 0, 1]])

cup2end = np.array([[1,  0, 0, 0],
                    [ 0, 1, 0, 0],
                    [ 0, 0, 1, -0.2535],
                    [ 0, 0, 0, 1]])

eye2end = np.dot(end2cam, cam2eye)
eye2end = np.linalg.inv(eye2end)
print(eye2end)

## target matrix
cup2eye = np.dot(np.dot(cup2end, end2cam), cam2eye)

eye.transform(np.dot(end2cam, cam2eye))
cup.transform(np.dot(end2cam, cam2eye))
cam.transform(end2cam)


o3d.visualization.draw_geometries([end, cam, eye, cup])

print(cup2eye)
