#pragma once

#include <string>
#include <vector>
#include <cuPhy/MathCommon.hh>

namespace mujoco::plugin::simplysoft
{
    struct SoftDef
    {
        struct DeformableObject
        {
            std::string name;
            double radius;
            cuVec3 fix_point1;
            cuVec3 fix_point2;
            cuVec3 dir_point;
            int dir_point_id;
            std::string bound_body;
            cuVec3 bound_pos1;
            cuVec3 bound_pos2;
            cuVec3 bound_dir_pos;
        };

        struct ObstracleObject
        {
            std::string name;
            double radius;
            std::string bound_body;
            cuVec3 bound_pos1;
            cuVec3 bound_pos2;
        };

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
        void LoadConfig(const std::string &config_path);
        SoftDef(const std::string &config_path);
        SoftDef() = default;
    };
}