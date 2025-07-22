#pragma once

#include <optional>
#include <vector>

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
  private:
    mjtNum                       scale;           ///< scale rate.
    bool                         show_wireframe;  ///< render settings.
    std::shared_ptr<Phy3DEngine> m_phy;           ///< simulate instance.
    std::shared_ptr<Phy3DScene>  m_scene;         ///< simulate scene, container.
    // mjtNum                       m_radius = 0.04; ///< robot's capsule radius.
    mjtNum                       m_radius = 0.0565; ///< robot's capsule radius.
    int sim_step = 5;
    bool barrier_valid = false;
    bool force_feedback = false;
    int barrier_id =0;

    std::vector<mjtNum> vert_pos_left;      ///< vertex cache for all tetraMesh (3*n).
    std::vector<mjtNum> vert_pos_right;     ///< vertex cache for all tetraMesh (3*n).
    std::vector<int>    surface_tris_left;  ///< triangle index for rendering (v0, v1, v2).
    std::vector<int>    surface_tris_right; ///< triangle index for rendering (v0, v1, v2).

    std::shared_ptr<Phy3DObstracle> robot_obstracle_left;  ///< [robot] simulate obstracle object.
    std::shared_ptr<SDFCapsule>     robot_sdf_left;        ///< [robot] simulate sdf object.
    std::shared_ptr<Phy3DObstracle> robot_obstracle_right; ///< [robot] simulate obstracle object.
    std::shared_ptr<SDFCapsule>     robot_sdf_right;       ///< [robot] simulate sdf object.

    cuType jt_scale      = 1.0;                           ///< [bind] use jointscale.
    cuType muscle_radius = 0.019 * jt_scale;              ///< [bind] collision capsule's radius
    cuVec3 l1pos         = cuVec3(0.143, 0.238, -0.007);  ///< [bind] for bind arm's init position.
    cuVec3 l2pos         = cuVec3(0.457, 0.217, -0.037);  ///< [bind] for bind arm's init position.
    cuVec3 r1pos         = cuVec3(-0.137, 0.232, -0.004); ///< [bind] for bind arm's init position.
    cuVec3 r2pos         = cuVec3(-0.466, 0.217, -0.040); ///< [bind] for bind arm's init position.

    std::shared_ptr<Phy3DObstracle>        human_obstracle_left;  ///< [human] simulate obstracle object.
    std::shared_ptr<SDFCapsule>            human_sdf_left;        ///< [human] simulate sdf object.
    std::shared_ptr<Phy3DObstracle>        human_obstracle_right; ///< [human] simulate obstracle object.
    std::shared_ptr<SDFCapsule>            human_sdf_right;       ///< [human] simulate sdf object.
    std::shared_ptr<Phy3DDeformableObject> tet_obj_left;          ///< [huamn] simulate deformable object.
    std::shared_ptr<Phy3DDeformableObject> tet_obj_right;         ///< [huamn] simulate deformable object.

    mjtNum* human_pos_left1;  // human's joint position.
    mjtNum* human_pos_left2;  // human's joint position.
    mjtNum* human_pos_left3;  // human's joint position.
    mjtNum* human_pos_right1; // human's joint position.
    mjtNum* human_pos_right2; // human's joint position.
    mjtNum* human_pos_right3; // human's joint position.

    mjtNum* robot_pos_left1;  // robot's joint position.
    mjtNum* robot_pos_left2;  // robot's joint position.
    mjtNum* robot_pos_left3;  // robot's joint position.
    mjtNum* robot_pos_right1; // robot's joint position.
    mjtNum* robot_pos_right2; // robot's joint position.
    mjtNum* robot_pos_right3; // robot's joint position.

    int lforce_num = 214; // left arm force sensor number.
    int rforce_num = 214; // right arm force sensor number.

    // cuVec3 init_offset = cuVec3(0, 2.5, -1.17);     // for bunny
    // cuVec3 init_offset = cuVec3(0.2, 1.1, -1.0);    // for arm and joint.
    cuVec3 init_offset = cuVec3(0.0, 1.1, 0.0);     // [used] for arm and joint.
    cuVec3 init_rot    = glm::vec3(0.0, pi_2, 0.0); // rotation

    cuVec3 total_force_left  = cuVec3(0.0, 0.0, 0.0);
    cuVec3 centroid_left     = cuVec3(0.0, 0.0, 0.0);
    cuVec3 total_force_right = cuVec3(0.0, 0.0, 0.0);
    cuVec3 centroid_right    = cuVec3(0.0, 0.0, 0.0);

    int body_id_upperarm_L_ = -1; 
    int body_id_upperarm_R_ = -1; 
    void applyForceToBodyChain(const mjModel* m, mjData* d, 
      int body_id,
      const cuVec3& F_world,
      const cuVec3& P_world);     

  public:
    SimpleSoft(const mjModel* m, mjData* d, int instance);
    ~SimpleSoft() = default;

    static std::optional<SimpleSoft> Create(const mjModel* m, mjData* d, int instance);
    static void                      RegisterPlugin();

    void Reset(const mjModel* m, int instance);
    void Compute(const mjModel* m, mjData* d, int instance);
    void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

  private:
    void DrawCapsule(const mjModel* m, mjvScene* scn, mjtNum radius, int jid, std::shared_ptr<SDFCapsule> cap, bool is_human = false);

    void BuildBindObstracles(std::shared_ptr<Phy3DEngine> phy);

    std::shared_ptr<Phy3DDeformableObject> LoadTetrahedralMesh(const std::string& scene_path, const std::string& config_name, std::vector<mjtNum>& vert_pos, std::vector<int>& surface_tris);

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
        // for (size_t i = 0; i < m_scene->tet_objects.size(); i++) {
        //     auto tet = m_scene->tet_objects[i];
        //     for (size_t i = 0; i < tet->cur_pos.size(); i++) {
        //         // std::cout << tet->cur_pos[i] << std::endl;
        //     }
        //     std::cout << tet->cur_pos[0] << "  " << tet->vel[0] << std::endl;
        // }

        // for (size_t i = 0; i < m_scene->obstracles.size(); i++) {
        //     std::shared_ptr<Phy3DObstracle>& m = m_scene->obstracles[i];
        //     std::cout << "    cur_pos: " << m->cur_pos << " " << m->vel << std::endl;
        // }
    }

    void SubStepSim() {
        // std::cout << "    substep.." << std::endl;
    }
};

} // namespace mujoco::plugin::simplysoft
