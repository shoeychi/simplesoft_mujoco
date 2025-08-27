#pragma once

#include <cuPhy/Phy3DEngine.hh>
#include <mujoco/mjtnum.h>
#include "SoftDef.hh"

namespace mujoco::plugin::hrpc
{
    struct ContactForces
    {
        int from_tet_id = 0;
        int to_sdf_id = 0;
        std::vector<cuVec3> contact_forces = {};
        std::vector<cuVec3> contact_points = {};

        cuVec3 total_contact_force = {0.0, 0.0, 0.0};
        cuVec3 total_contact_force_pos = {0.0, 0.0, 0.0};
        cuVec3 total_contact_torque = {0.0, 0.0, 0.0}; // 寻找0力矩点，构成力螺旋

        void clear();
        void update_total_force();
        ContactForces &operator=(const ContactForces &o);
    };

    struct PhyscisEngineData
    {

        std::shared_ptr<Phy3DEngine> m_phy_;  ///< simulate instance.
        std::shared_ptr<Phy3DScene> m_scene_; ///< simulate scene, container.

        std::vector<std::shared_ptr<SDFCapsule>> robot_sdf_;
        std::vector<std::shared_ptr<Phy3DObstracle>> robot_obstracles_;

        std::vector<std::shared_ptr<SDFCapsule>> human_sdf_;
        std::vector<std::shared_ptr<Phy3DObstracle>> human_obstracles_;

        std::vector<std::shared_ptr<Phy3DDeformableObject>> tet_obj_;
        std::vector<std::vector<std::array<mjtNum, 3>>> vertices_pos_;
        std::vector<std::vector<std::array<int, 3>>> surface_trangles_;
        std::vector<std::vector<std::array<int, 2>>> edges_;

        std::vector<std::vector<ContactForces>> contact_forces_;
        std::vector<double> tet_rotation_ang_;

        explicit PhyscisEngineData(std::shared_ptr<SoftDef> &softDef_);
        std::shared_ptr<SoftDef> softDef_;

    private:
        void Initializeobstracles();
        void InitializeDeformable();
        std::shared_ptr<Phy3DDeformableObject> LoadTetrahedralMesh(const std::string &scene_path,
                                                                   const std::string &config_name,
                                                                   std::vector<std::array<mjtNum, 3>> &vert_pos,
                                                                   std::vector<std::array<int, 3>> &surface_tris,
                                                                   std::vector<std::array<int, 2>> &edges);
    };
}