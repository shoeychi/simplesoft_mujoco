#include "SoftDef.hh"
#include <fstream>
#include <nlohmann/json.hpp>

namespace mujoco::plugin::simplysoft
{
    void SoftDef::LoadConfig(const std::string &config_path)
    {
        std::ifstream config_file(config_path);
        if (!config_file.good())
        {
            std::cout << "load config file failed.\n File = " << config_path << std::endl;
            return;
        }
        nlohmann::json config = nlohmann::json::parse(config_file);

        model_path_ = config["model_path"];
        barrier_valid_ = config["barrier_valid"];
        force_feedback_ = config["force_feedback"];
        show_contact_force_ = config["show_contact_force"];
        show_total_contact_force_ = config["show_total_contact_force"];
        show_sdf_label_ = config["show_sdf_label"];
        sim_step_ = config["sim_step"];
        force_scale_ = config["force_scale"];
        force_vis_scale_ = config["force_vis_scale"];
        max_geom_ = config["maxgeom"];

        for (const auto &obj_cfg : config["deformable_objects"])
        {
            DeformableObject object;
            object.name = obj_cfg["name"];
            object.radius = obj_cfg["radius"];
            std::vector<double> p1 = obj_cfg["fix_point1"];
            std::vector<double> p2 = obj_cfg["fix_point2"];
            std::vector<double> pd = obj_cfg["dir_point"];
            object.fix_point1 = cuVec3(p1[0], p1[1], p1[2]);
            object.fix_point2 = cuVec3(p2[0], p2[1], p2[2]);
            object.dir_point = cuVec3(pd[0], pd[1], pd[2]);
            object.dir_point_id = obj_cfg["dir_point_id"];
            object.bound_body = obj_cfg["bound_body"];
            std::vector<double> p3 = obj_cfg["bound_pos1"];
            std::vector<double> p4 = obj_cfg["bound_pos2"];
            std::vector<double> p5 = obj_cfg["bound_dir_pos"];
            object.bound_pos1 = cuVec3(p3[0], p3[1], p3[2]);
            object.bound_pos2 = cuVec3(p4[0], p4[1], p4[2]);
            object.bound_dir_pos = cuVec3(p5[0], p5[1], p5[2]);
            deformable_objects_.push_back(object);
        }
        num_deformable_objects_ = deformable_objects_.size();

        for (const auto &obj_cfg : config["obstracles"])
        {
            ObstracleObject object;
            object.name = obj_cfg["name"];
            object.radius = obj_cfg["radius"];
            object.bound_body = obj_cfg["bound_body"];
            std::vector<double> p3 = obj_cfg["bound_pos1"];
            std::vector<double> p4 = obj_cfg["bound_pos2"];
            object.bound_pos1 = cuVec3(p3[0], p3[1], p3[2]);
            object.bound_pos2 = cuVec3(p4[0], p4[1], p4[2]);
            obstracle_objects_.push_back(object);
        }
        num_obstracles_ = obstracle_objects_.size();
    }

    SoftDef::SoftDef(const std::string &config_path)
    {
        LoadConfig(config_path);
    }
}