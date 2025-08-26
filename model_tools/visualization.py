import pyvista as pv
import numpy as np
import sys
from scipy.spatial.distance import cdist

class Visualization:
    def __init__(self, path: str, model_name:str):
        """
        通过可视化验证生成的四面体网格
        """

        self.path = path
        self.model_name = model_name
    
        self.normal = [0., 0., 1.]
    
        self.read_points()  
        self.read_faces()   
        self.read_elements()   
        
        
        
    def read_points(self): 
        file_name = self.path + '/' + self.model_name + '.1.node'
        data_list = []
        data_id = []
        data_marker = []
        with open(file_name, 'r') as file:
            for line in file:
                # 去除行尾换行符并按空格分割
                data_str = line.strip().split()
                if not data_str[0]=="#" and len(data_str)==5:
                    data = [float(data_str[1]), float(data_str[2]), float(data_str[3])]
                    data_list.append(data)
                    data_id.append(int(data_str[0]))
                    data_marker.append(int(data_str[4]))
                
        self.points = np.array(data_list)
        self.points_markers = np.array(data_marker)
        self.points_colors = np.zeros((len(self.points), 3), dtype=float)
        self.points_colors[self.points_markers==1, 0] = 1
        self.points_colors[self.points_markers==2, 2] = 1
        self.points_cloud = pv.PolyData(self.points)
        self.center = np.mean(self.points, axis=0)
        
        # 找到最远两个marker=2的点的坐标
        self.points_fix = self.points[self.points_markers==2]
        dist_matrix = cdist(self.points_fix, self.points_fix)
        np.fill_diagonal(dist_matrix, 0)  # 将对角线设为0以避免自比较
        max_idx = np.unravel_index(np.argmax(dist_matrix), dist_matrix.shape)
        point1 = self.points_fix[max_idx[0]]
        point2 = self.points_fix[max_idx[1]]
        max_distance = dist_matrix[max_idx]

        print("Recommended point 1 to fix deformable to sdf: ", point1)
        print("Recommended point 2 to fix deformable to sdf: ", point2)
        print("Distance is: ", max_distance)
        
        # 找到额外一个marker=2的点的坐标，用于更新方向
        vec0 = point2 - point1
        vec = self.points_fix - point1
        vec_cross = np.linalg.cross(vec, vec0)
        vec_cross_n = np.linalg.norm(vec_cross, axis=1)
        dis = vec_cross_n / np.linalg.norm(vec0)
        point3 = self.points_fix[np.argmax(dis)]
        point3_id = np.where((self.points == point3).all(axis=1))[0]
        print("Recommended point 3 to fix deformable to sdf: ", point3)
        print("Id of point 3: ", point3_id)
        
    
    def read_faces(self): 
        file_name = self.path + '/' + self.model_name + '.1.face'
        data_list = []
        data_id = []
        data_marker = []
        with open(file_name, 'r') as file:
            for line in file:
                # 去除行尾换行符并按空格分割
                data_str = line.strip().split()
                if not data_str[0]=="#" and len(data_str)==5:
                    data = [int(data_str[1]), int(data_str[2]), int(data_str[3])]
                    data_list.append(data)
                    data_id.append(int(data_str[0]))
                    data_marker.append(int(data_str[4]))
                
        self.faces = np.array(data_list, dtype=int)
        self.faces_markers = np.array(data_marker)
        self.faces_aug = np.hstack([np.ones((len(self.faces), 1))*3, self.faces]).astype(np.int32)
        self.faces_mesh = pv.PolyData(self.points, self.faces_aug)
        
        self.faces_colors = np.zeros((len(self.faces), 3), dtype=float)
        self.faces_colors[self.faces_markers==1, 0] = 1  # 外部点为红色
        self.faces_colors[self.faces_markers==2, 2] = 1  # 内部点为蓝色
        self.faces_mesh.cell_data["colors"] = self.faces_colors
        
        self.faces_mesh_clip = self.faces_mesh.clip(normal=self.normal, origin=self.center)
        
    def read_elements(self): 
        file_name = self.path + '/' + self.model_name + '.1.ele'
        data_list = []
        data_id = []
        data_marker = []
        with open(file_name, 'r') as file:
            for line in file:
                # 去除行尾换行符并按空格分割
                data_str = line.strip().split()
                if not data_str[0]=="#" and len(data_str)==6:
                    data = [int(data_str[1]), int(data_str[2]), int(data_str[3]), int(data_str[4])]
                    data_list.append(data)
                    data_id.append(int(data_str[0]))
                    data_marker.append(int(data_str[5]))
                
        self.elements = np.array(data_list, dtype=int)
        self.ele_aug = np.hstack(
                [np.ones((len(self.elements), 1))*4, self.elements]
            ).astype(np.int32)
        self.ele_mesh = pv.UnstructuredGrid(
                self.ele_aug,
                [pv.CellType.TETRA]*len(self.ele_aug),
                self.points
            )
        self.ele_mesh_clip = self.ele_mesh.clip(normal=self.normal, origin=self.center)
        
        
    def show(self):
        plotter1 = pv.Plotter()
        plotter1.add_points(self.points_cloud, scalars=self.points_colors, rgb=True)
        plotter1.show(auto_close=False)

        plotter2 = pv.Plotter()
        plotter2.add_mesh(self.faces_mesh, scalars="colors", rgb=True, show_edges=True)
        plotter2.show(auto_close=False)
        
        plotter3 = pv.Plotter()
        plotter3.add_mesh(self.faces_mesh_clip, scalars="colors", rgb=True, show_edges=True)
        plotter3.show(auto_close=False)
        
        plotter4 = pv.Plotter()
        plotter4.add_mesh(self.ele_mesh, show_edges=True)
        plotter4.show(auto_close=False)
        
        plotter5 = pv.Plotter()
        plotter5.add_mesh(self.ele_mesh_clip, show_edges=True)
        plotter5.show()
        

if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise ValueError("请输入模型名称")
    
    path = str(sys.argv[1])
    model_name = str(sys.argv[1])
    
    vis = Visualization(path, model_name)
    vis.show()
    