#pragma once

#include <optional>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include "Phy3DEngine.hh"
#include "Phy3DHelper.hh"

#include <memory>

#define pi 3.14159265358979323846
#define pi_2 1.57079632679489661923

namespace mujoco::plugin::simplysoft {

class SimpleSoft {
  public:
    SimpleSoft(const mjModel* m, mjData* d, int instance);
    ~SimpleSoft() = default;

    static void RegisterPlugin();

    void Reset(const mjModel* m, int instance);
    void Compute(const mjModel* m, mjData* d, int instance);
    void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

  private:
    void LoadConfig(const std::string& config_path);
    void Initializeobstracles();
    void InitializeDeformable();
    std::shared_ptr<Phy3DDeformableObject> LoadTetrahedralMesh(
      const std::string& scene_path, 
      const std::string& config_name, 
      std::vector<std::array<mjtNum, 3>>& vert_pos, 
      std::vector<std::array<int, 3>>& surface_tris
    );
    cuVec3 GetBodyPoseForSdf(const mjModel* m, const mjData* d, const std::string& body_name, const cuVec3& pos_at_body);
    void DrawCapsule(const mjModel* m, mjvScene* scn, mjtNum radius, int lable_id, std::shared_ptr<SDFCapsule> sdf, float* rgba);
    bool ContactPoint2Sdf(const cuVec3& p, const cuVec3& a, const cuVec3& b, double r);

    
    // UniX -> MuJoCo 坐标系转换工具函数, 将Y-Z轴互换
    void UniX2MuJoCO(cuVec3& vec) {
        std::swap(vec.y, vec.z); // Y-Z轴转换
    }

    void InitSim() {
        // std::cout << "Init.." << std::endl;
    }

    void StartSim() {
        // std::cout << "  StartSim" << std::endl;
    }

    void StepSim() {
        // std::cout << "  step.." << std::endl;
    }

    void SubStepSim() {
        // std::cout << "    substep.." << std::endl;
    }

    struct DeformableObject
    {
      std::string name;
      double radius;
      cuVec3 fix_point1;
      cuVec3 fix_point2;
      std::string bound_body;
      cuVec3 bound_pos1;
      cuVec3 bound_pos2;
    };
    
    struct ObstracleObject
    {
      std::string name;
      double radius;
      std::string bound_body;
      cuVec3 bound_pos1;
      cuVec3 bound_pos2;
    };

    std::shared_ptr<Phy3DEngine> m_phy_;           ///< simulate instance.
    std::shared_ptr<Phy3DScene>  m_scene_;         ///< simulate scene, container.

    std::string model_path_;
    bool barrier_valid_ = false;
    bool force_feedback_ = false;
    bool show_contact_force_ = true;
    bool show_total_contact_force_ = true;
    bool show_sdf_label_ = false;
    int sim_step_ = 1;
    double force_scale_ = 1.0;
    double force_vis_scale_ = 1.0;
    int max_geom_;
    

    std::vector<DeformableObject> deformable_objects_;
    std::vector<ObstracleObject> obstracle_objects_;
    int num_deformable_objects_ = 0;
    int num_obstracles_ = 0;
    std::vector<std::shared_ptr<SDFCapsule>> robot_sdf_;
    std::vector<std::shared_ptr<Phy3DObstracle>> robot_obstracles_;
    
    std::vector<std::shared_ptr<Phy3DDeformableObject>> tet_obj_;
    std::vector<std::vector<std::array<mjtNum, 3>>> vertices_pos_;
    std::vector<std::vector<std::array<int, 3>>> surface_trangles_;

    std::vector<std::shared_ptr<SDFCapsule>> human_sdf_;
    std::vector<std::shared_ptr<Phy3DObstracle>> human_obstracles_;
    
    struct ContactForces{
        //注意全部以mujoco坐标系为准
        int from_tet_id = 0;
        int to_sdf_id = 0;
        std::vector<cuVec3> contact_forces = {};  
        std::vector<cuVec3> contact_points = {};

        cuVec3 total_contact_force = {0.0, 0.0, 0.0};
        cuVec3 total_contact_force_pos = {0.0, 0.0, 0.0};
        cuVec3 total_contact_torque = {0.0, 0.0, 0.0}; //寻找0力矩点，构成力螺旋

        void clear(){
            contact_forces.clear();
            contact_points.clear();
        }
        
        void update_total_force();
        // void to_mj_body(const cuVec3& body_pos, bool is_robot);
    };
    
    std::vector<std::vector<ContactForces>> contact_forces_;

    
};

} // namespace mujoco::plugin::simplysoft
