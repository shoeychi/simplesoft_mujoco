# 准备软体模型

### (1) 几何模型和3D网格划分

准备对齐的骨骼和皮肤的stl几何模型，如图所示。

所需的可变形体，是skin模型和skeleton模型的差，也就是单个stl模型，表面为皮肤形状，内部空洞为骨骼形状。

在仿真中，会识别内部空洞的节点，并施以固定约束。

<div style="text-align:center">
    <img src="../doc/示意图1.png" alt="" style="width:30%"/>
</div>

准备上述stl格式文件后，按如下步骤准备：

0. 将stl复制到一个单一文件夹中，文件夹命名为模型名称，后续所有调用以文件夹的名称作为模型唯一名称

1. 加载stl模型，识别节点，获得poly格式模型（将外表面节点mark为1，和骨骼接触的内表面节点mark为2）。

   ```
   python model_generation.py   # 需要修改代码中的文件和模型名称
   ```
2. 运行tetgen生成3D网格模型。（建议软体模型节点数量在8000以下，加q会进行细化处理，不建议）
   ```
   tetgen -pAqz test_model.poly
   tetgen -pAz test_model.poly
   ```
3. 运行脚本将表面节点提取出来，得到json文件用于提取接触力
   ```
   python generate_samples.py <model_name>
   ```

4. 采用可视化工具检查模型的正确性。记录打印的关键点坐标，将用于变形体和刚体的绑定。
   ```
   python visualization.py <model_name>
   ```

5. 写一个软体仿真的配置文件，修改对应的模型名称，其他建议不要改动。
   可参考：test_model.yaml

6. 经过上述步骤，得到模型文件包括（以测试模型为例）：
    * test_model.1.node
    * test_model.1.face
    * test_model.1.ele
    * test_model.1.edge
    * test_model.json
    * test_model.yaml

    运行以下命令将模型文件复制到软体求解器可调用的路径
    ```
    python copy_model.py <model_name>
    ```
    具体会实现将：
    * 将模型复制到/usr/share/biulab/files/
    * 将配置文件复制到/usr/share/biulab/config/
    * 将json文件复制到/usr/share/biulab/resources/models/

7. 运行以下命令，在UI界面load config查看模型是否有其他问题
  ```
  Phy3DDemo
  ```

经过上述步骤，才能在mjcf中包含该软体模型，从而进行仿真。
