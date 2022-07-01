import open3d as o3d
import numpy as np

mesh = o3d.geometry.TriangleMesh.create_box(0.1, 0.1, 0.1)
mesh.compute_vertex_normals()
mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
coor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.2)
# o3d.visualization.draw_geometries([mesh, coor])

scene = o3d.t.geometry.RaycastingScene()
cube_id = scene.add_triangles(mesh_t)
print(cube_id)

# rays = o3d.core.Tensor([[0.05, -0.1, 0.05, 0, 1, 0]], dtype=o3d.core.Dtype.Float32)
rays = o3d.core.Tensor([[0.05, 0.05, -0.1, 0, 0, 1]], dtype=o3d.core.Dtype.Float32)

ans = scene.cast_rays(rays)
for k in list(ans.keys()):
    print("{}:{}".format(k, ans[k]))
    print()