import numpy as np
import json
import sys

class GenerateSamples:
    def __init__(self, model_name):
        self.model_name = model_name
        self.read_points()

    def read_points(self): 
        file_name = self.model_name + '/' + self.model_name + '.1.node'
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
        self.markers = np.array(data_marker)

        self.sample_points = self.points[self.markers==1]
        self.sample_ids = np.where(self.markers == 1)[0]
        
    def to_json(self):
        json_config = dict()
        json_config["object_name"] = self.model_name
        json_config["selected_vertex_count"] = len(self.sample_ids)
        json_config["vertices"] = []
        for i in range(json_config["selected_vertex_count"]):
            vertex = dict()
            vertex["index"] = int(self.sample_ids[i])
            vertex["position"] = self.sample_points[i].tolist()
            json_config["vertices"].append(vertex)
        
        file_name = self.model_name + '/' + self.model_name + '.json'
        with open(file_name, 'w', encoding='utf-8') as f:
            json.dump(json_config, f, ensure_ascii=False, indent=2)  # 保持中文可读

        print("Generate sample points file: "+file_name)

if __name__ == "__main__":
    
    if len(sys.argv) != 2:
        raise ValueError("请输入模型名称")
    
    model_name = str(sys.argv[1])
    
    generator = GenerateSamples(model_name)
    generator.to_json()
