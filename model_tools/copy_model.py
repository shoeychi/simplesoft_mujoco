import shutil
import sys

def copy_model_to_biulab(model_name: str):
    file_name1 = model_name + "/" + model_name + ".1.node"
    file_name2 = model_name + "/" + model_name + ".1.face"
    file_name3 = model_name + "/" + model_name + ".1.edge"
    file_name4 = model_name + "/" + model_name + ".1.ele"
    file_name5 = model_name + "/" + model_name + ".yaml"
    file_name6 = model_name + "/" + model_name + ".json"
    
    model_path = "/usr/share/biulab/files/"
    config_path = "/usr/share/biulab/config/"
    json_path = "/usr/share/biulab/resources/models/"
    
    shutil.copy(file_name1, model_path)
    print("move '" + file_name1 + "' to '" + model_path + "'")
    
    shutil.copy(file_name2, model_path)
    print("move '" + file_name2 + "' to '" + model_path + "'")
    
    shutil.copy(file_name3, model_path)
    print("move '" + file_name3 + "' to '" + model_path + "'")
    
    shutil.copy(file_name4, model_path)
    print("move '" + file_name4 + "' to '" + model_path + "'")
    
    shutil.copy(file_name5, config_path)
    print("move '" + file_name5 + "' to '" + config_path + "'")
    
    shutil.copy(file_name6, json_path)
    print("move '" + file_name6 + "' to '" + json_path + "'")
    
    
if len(sys.argv) != 2:
    raise ValueError("请输入模型名称")
    
model_name = str(sys.argv[1])
copy_model_to_biulab(model_name)