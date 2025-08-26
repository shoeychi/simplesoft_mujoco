import numpy as np
import pyvista as pv
import trimesh
# from mesh import Mesh


class ModelGeneration:
    def __init__(self, mesh_skin: str, mesh_skeleton: str, mesh_deformable=None):
        """
        模型生成器，生成用于人体仿真的几何模型
        """
        if mesh_deformable is None:
            self.mesh_skin = trimesh.load(mesh_skin)
            self.mesh_skeleton = trimesh.load(mesh_skeleton)
            self.mesh_deformable = trimesh.boolean.difference([self.mesh_skin.trimesh, self.mesh_skeleton.trimesh])
        else:
            self.mesh_deformable = trimesh.load(mesh_deformable)

        self.update_point_markers()

    def update_point_markers(self, cutoff_distance=1.0):
        points = self.mesh_deformable.vertices
        normals = self.mesh_deformable.vertex_normals
        
        ray_mesh = trimesh.ray.ray_triangle.RayMeshIntersector(self.mesh_deformable)
        intersects = ray_mesh.intersects_any(points+0.0001*normals, normals)
        # locations = ray_mesh.intersects_location(points+0.0001*normals, normals)
        self.point_markers = np.ones(len(points), dtype=int)
        self.point_markers[intersects==True] = 2
        
    def to_poly(self, file_name: str):
        with open(file_name, 'w', encoding='utf-8') as f:
            f.write("# Part 1 - node list\n")
            f.write(
                "# <# of points> <dimension (3)> <# of attributes> <boundary markers (0 or 1)>\n")
            num_points = len(self.mesh_deformable.vertices)
            f.write(str(num_points) + " 3 0 1\n")
            f.write("# <point #> <x> <y> <z> [attributes] [boundary marker]\n")

            for i in range(num_points):
                f.write(str(i+1)+" ")
                f.write(str(self.mesh_deformable.vertices[i, 0])+" ")
                f.write(str(self.mesh_deformable.vertices[i, 1])+" ")
                f.write(str(self.mesh_deformable.vertices[i, 2])+" ")

                f.write(str(self.point_markers[i])+"\n")

            f.write("# Part 2 - facet list\n")
            f.write("# <# of facets> <boundary markers (0 or 1)>\n")
            num_faces = len(self.mesh_deformable.faces)
            f.write(str(num_faces) + " 1\n")
            f.write("# <# of corners> <corner 1> <corner 2> ... <corner #>\n")

            self.face_markers = np.ones(num_faces)
            for i in range(num_faces):
                triangle_point_markers = self.point_markers[self.mesh_deformable.faces[i]]
                if np.any(triangle_point_markers==2):
                    triangle_marker = 2
                    self.face_markers[i] = 2
                else:
                    triangle_marker = 1

                f.write("1 0 "+str(triangle_marker)+"\n")
                f.write("3 ")
                f.write(str(self.mesh_deformable.faces[i, 0]+1) + " ")
                f.write(str(self.mesh_deformable.faces[i, 1]+1) + " ")
                f.write(str(self.mesh_deformable.faces[i, 2]+1) + "\n")
            
            f.write("# Part 3 - hole list\n")
            f.write("0\n")
            f.write("# Part 4 - region list\n")
            f.write("0\n")
            f.write("# <region #> <x> <y> <z> <region attribute> <region volume constraint>\n")
        print("model '" + file_name + "' has been generated!")

    def visualization_test(self):
        # 画出点
        points = self.mesh_deformable.vertices.copy()
        point_cloud = pv.PolyData(points)
        
        point_colors = np.zeros((len(points), 3), dtype=float)
        point_colors[self.point_markers==1, 0] = 1  # 外部点为红色
        point_colors[self.point_markers==2, 2] = 1  # 内部点为蓝色
        
        # 画出面
        num_faces = len(self.mesh_deformable.faces)
        elements_aug = np.hstack([np.ones((num_faces, 1))*3, self.mesh_deformable.faces]).astype(np.int32)
        mesh = pv.PolyData(points, elements_aug)
        
        face_colors = np.zeros((num_faces, 3), dtype=float)
        face_colors[self.face_markers==1, 0] = 1  # 外部点为红色
        face_colors[self.face_markers==2, 2] = 1  # 内部点为蓝色
        mesh.cell_data["colors"] = face_colors
        
        plotter1 = pv.Plotter()
        plotter1.add_points(point_cloud, scalars=point_colors, rgb=True)
        plotter1.show(auto_close=False)

        plotter2 = pv.Plotter()
        plotter2.add_mesh(mesh, scalars="colors", rgb=True, show_edges=True)
        plotter2.show()
        
if __name__ == "__main__":
    mesh_skin = "test_model/skin.stl"
    mesh_skeleton = "test_model/skeleton.stl"
    mesh_deformable = "human_neck/skin_neck.stl"

    mesh_gen = ModelGeneration(mesh_skin, mesh_skeleton, mesh_deformable)

    file_name = "human_neck/human_neck.poly"
    mesh_gen.to_poly(file_name)

    mesh_gen.visualization_test()