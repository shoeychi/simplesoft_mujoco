#include <algorithm>
#include <cstddef>
#include <optional>
#include <iostream>

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

#include "HumanRobotPhysicalContact.hh"
#include "SoftHelper.hh"

namespace mujoco::plugin::hrpc
{
    void HumanRobotPhysicalContact::RegisterPlugin()
    {
        mjpPlugin plugin;
        mjp_defaultPlugin(&plugin);

        plugin.name = "mujoco.HumanRobotPhysicalContact";
        plugin.capabilityflags |= mjPLUGIN_PASSIVE;

        // 增加模型相关配置参数
        const char *attributes[] = {"config"};
        plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
        plugin.attributes = attributes;
        plugin.nstate = +[](const mjModel *m, int instance)
        { return 0; };

        plugin.init = +[](const mjModel *m, mjData *d, int instance)
        {
            // 无法避免初始化两次，第一次是检查
            d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new HumanRobotPhysicalContact(m, instance));
            return 0;
        };

        // 注册状态更新回调
        plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data, int instance)
        {
            auto *soft = reinterpret_cast<HumanRobotPhysicalContact *>(plugin_data);
            soft->Reset(m, instance);
        };

        plugin.destroy = +[](mjData *d, int instance)
        {
            delete reinterpret_cast<HumanRobotPhysicalContact *>(d->plugin_data[instance]);
            d->plugin_data[instance] = 0;
        };

        plugin.compute = +[](const mjModel *m, mjData *d, int instance, int capability_bit)
        {
            auto *elasticity = reinterpret_cast<HumanRobotPhysicalContact *>(d->plugin_data[instance]);
            elasticity->Compute(m, d, instance);
        };

        plugin.visualize = +[](const mjModel *m, mjData *d, const mjvOption *opt, mjvScene *scn,
                               int instance)
        {
            auto *elasticity = reinterpret_cast<HumanRobotPhysicalContact *>(d->plugin_data[instance]);
            elasticity->Visualize(m, d, scn, instance);
        };

        mjp_registerPlugin(&plugin);
    }

    PhyscisEngineData *HumanRobotPhysicalContact::phyEngData_ = nullptr;
    HumanRobotPhysicalContact::HumanRobotPhysicalContact(const mjModel *m, int instance)
    {
        // 读取配置文件
        if (phyEngData_ == nullptr)
        {
            std::shared_ptr<SoftDef> softDef_ = std::make_shared<SoftDef>();
            std::string config_file = mj_getPluginConfig(m, instance, "config");
            softDef_->LoadConfig(config_file);
            phyEngData_ = new PhyscisEngineData(softDef_);
            phyEngData_->m_phy_->registerInitCallback(this, &HumanRobotPhysicalContact::InitSim);
            phyEngData_->m_phy_->registerStartSimulateCallback(this, &HumanRobotPhysicalContact::StartSim);
            phyEngData_->m_phy_->registerStepCallback(this, &HumanRobotPhysicalContact::StepSim);
            phyEngData_->m_phy_->registerSubStepCallback(this, &HumanRobotPhysicalContact::SubStepSim);
        }
    }

    void HumanRobotPhysicalContact::Reset(const mjModel *m, int instance) {}

    void HumanRobotPhysicalContact::Compute(const mjModel *m, mjData *d, int instance)
    {
        static int compute_counter = -1;
        compute_counter++;

        // std::cout << "Call compute, step = " << compute_counter << std::endl;

        // update robot and human's joint position.
        if (instance && m && d)
        {
            mj_forward(m, d);
        }

        if (compute_counter % phyEngData_->softDef_->sim_step_ != 0)
            return;

        // 执行软体pbd仿真
        phyEngData_->m_phy_->config->step_one = true;
        cuType dt = m->opt.timestep * phyEngData_->softDef_->sim_step_;
        phyEngData_->m_phy_->SimlateStep(dt);

        // 更新human sdf位置
        for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
        {
            cuVec3 pos1 = GetBodyPoseForSdf(m, d, phyEngData_->softDef_->deformable_objects_[i].bound_body, phyEngData_->softDef_->deformable_objects_[i].bound_pos1);
            cuVec3 pos2 = GetBodyPoseForSdf(m, d, phyEngData_->softDef_->deformable_objects_[i].bound_body, phyEngData_->softDef_->deformable_objects_[i].bound_pos2);
            phyEngData_->human_sdf_[i]->a = pos1; // set position,
            phyEngData_->human_sdf_[i]->b = pos2; // set position,

            cuVec3 pos_dir_act = phyEngData_->tet_obj_[i]->cur_pos[phyEngData_->softDef_->deformable_objects_[i].dir_point_id];
            cuVec3 pos_dir = GetBodyPoseForSdf(m, d, phyEngData_->softDef_->deformable_objects_[i].bound_body, phyEngData_->softDef_->deformable_objects_[i].bound_dir_pos);

            double ang = GetRotationAngle(pos1, pos2, pos_dir, pos_dir_act);
            phyEngData_->tet_rotation_ang_[i] += ang;
            phyEngData_->human_sdf_[i]->ang = phyEngData_->tet_rotation_ang_[i];
        }

        // 更新robot sdf位置
        for (int i = 0; i < phyEngData_->softDef_->num_obstracles_; ++i)
        {
            cuVec3 pos1 = GetBodyPoseForSdf(m, d, phyEngData_->softDef_->obstracle_objects_[i].bound_body, phyEngData_->softDef_->obstracle_objects_[i].bound_pos1);
            cuVec3 pos2 = GetBodyPoseForSdf(m, d, phyEngData_->softDef_->obstracle_objects_[i].bound_body, phyEngData_->softDef_->obstracle_objects_[i].bound_pos2);
            phyEngData_->robot_sdf_[i]->a = pos1; // set position,
            phyEngData_->robot_sdf_[i]->b = pos2; // set position,
        }

        // 更新接触力
        for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
        {
            for (int j = 0; j < phyEngData_->softDef_->num_obstracles_; ++j)
            {
                phyEngData_->contact_forces_[i][j].clear();
            }
        }
        for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
        {
            auto forces = phyEngData_->m_phy_->m_phy_scene->tet_objects[i]->sampled_force;
            for (size_t j = 0; j < forces.size(); ++j)
            {
                auto pos = forces[j].pos;
                auto f = forces[j].f;
                for (int k = 0; k < phyEngData_->softDef_->num_obstracles_; ++k)
                {
                    auto a = phyEngData_->robot_sdf_[k]->a;
                    auto b = phyEngData_->robot_sdf_[k]->b;
                    double r = phyEngData_->robot_sdf_[k]->r;
                    if (ContactPoint2Sdf(pos, a, b, r))
                    {
                        phyEngData_->contact_forces_[i][k].contact_forces.push_back(f * phyEngData_->softDef_->force_scale_);
                        phyEngData_->contact_forces_[i][k].contact_points.push_back(pos);
                    }
                }
            }
        }
        for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
        {
            for (int j = 0; j < phyEngData_->softDef_->num_obstracles_; ++j)
            {
                phyEngData_->contact_forces_[i][j].update_total_force();
            }
        }

        // 施加接触力到mujoco body上（TODO：有问题，加的力似乎是不断叠加的）
        if (phyEngData_->softDef_->force_feedback_)
        {
            for (int i = 0; i < m->nv; ++i)
            {
                d->qfrc_applied[i] = 0.0; // 先把外力置0，否则会持续叠加，这里会不会有问题，TODO：待重力测试
            }

            for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
            {
                for (int j = 0; j < phyEngData_->softDef_->num_obstracles_; ++j)
                {
                    auto f = phyEngData_->contact_forces_[i][j].total_contact_force;
                    auto t = phyEngData_->contact_forces_[i][j].total_contact_torque;
                    auto p = phyEngData_->contact_forces_[i][j].total_contact_force_pos;
                    int rob_body_id = mj_name2id(m, mjOBJ_BODY, phyEngData_->softDef_->obstracle_objects_[j].bound_body.c_str());
                    int hum_body_id = mj_name2id(m, mjOBJ_BODY, phyEngData_->softDef_->deformable_objects_[i].bound_body.c_str());

                    mjtNum force[3] = {f.x, f.y, f.z};
                    mjtNum torque[3] = {t.x, t.y, t.z};
                    mjtNum point[3] = {p.x, p.y, p.z};
                    mj_applyFT(m, d, force, torque, point, hum_body_id, d->qfrc_applied);

                    mjtNum force2[3] = {-f.x, -f.y, -f.z};
                    mjtNum torque2[3] = {-t.x, -t.y, -t.z};
                    mj_applyFT(m, d, force2, torque2, point, rob_body_id, d->qfrc_applied);
                }
            }
        }
    }

    double HumanRobotPhysicalContact::GetRotationAngle(const cuVec3 &pos1, const cuVec3 &pos2, const cuVec3 &pos_dir1, const cuVec3 &pos_dir2)
    {
        mjtNum vec12[3] = {pos2.x - pos1.x, pos2.y - pos1.y, pos2.z - pos1.z};
        mjtNum vec1d1[3] = {pos_dir1.x - pos1.x, pos_dir1.y - pos1.y, pos_dir1.z - pos1.z};
        mjtNum vec1d2[3] = {pos_dir2.x - pos1.x, pos_dir2.y - pos1.y, pos_dir2.z - pos1.z};
        mjtNum n1[3];
        mjtNum n2[3];
        mju_cross(n1, vec12, vec1d1);
        mju_cross(n2, vec12, vec1d2);
        mju_normalize3(n1);
        mju_normalize3(n2);
        double ang = std::acos(mju_dot3(n1, n2));

        mjtNum n_cross[3];
        mju_cross(n_cross, n1, n2);
        mjtNum d = mju_dot3(n_cross, vec12);
        if (d > 0)
        {
            ang = -ang;
        }
        return ang;
    }

    cuVec3 HumanRobotPhysicalContact::GetBodyPoseForSdf(const mjModel *m, const mjData *d, const std::string &body_name, const cuVec3 &pos_at_body)
    {
        int body_id = mj_name2id(m, mjOBJ_BODY, body_name.c_str());
        mjtNum *body_pos = d->xpos + 3 * body_id;
        mjtNum *body_mat = d->xmat + 9 * body_id;
        mjtNum local_pos[3] = {pos_at_body[0], pos_at_body[1], pos_at_body[2]};
        mjtNum rot_pos[3] = {0.0, 0.0, 0.0};
        mju_mulMatVec3(rot_pos, body_mat, local_pos);
        mjtNum pos[3] = {0.0, 0.0, 0.0};
        mju_add3(pos, body_pos, rot_pos);
        cuVec3 p = {pos[0], pos[1], pos[2]};
        return p;
    }

    bool HumanRobotPhysicalContact::ContactPoint2Sdf(const cuVec3 &p, const cuVec3 &a, const cuVec3 &b, double r)
    {
        // 求投影点 c, 首先有
        double t1 = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y) + (p.z - a.z) * (b.z - a.z);
        double t2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z);
        double t = t1 / t2;

        double xc = a.x + t * (b.x - a.x);
        double yc = a.y + t * (b.y - a.y);
        double zc = a.z + t * (b.z - a.z);

        mjtNum P[3] = {p.x, p.y, p.z};
        mjtNum c[3] = {xc, yc, zc};
        mjtNum A[3] = {a.x, a.y, a.z};
        mjtNum B[3] = {b.x, b.y, b.z};

        mjtNum pc = mju_dist3(P, c);
        mjtNum ac = mju_dist3(A, c);
        mjtNum bc = mju_dist3(B, c);
        mjtNum ab = mju_dist3(A, B);
        mjtNum R = r;
        mjtNum err = 0.001;
        if (pc > R + err)
            return false;
        if (ac > ab + R + err)
            return false;
        if (bc > ab + R + err)
            return false;
        return true;
    }

    void HumanRobotPhysicalContact::Visualize(const mjModel *m, mjData *d, mjvScene *scn, int instance)
    {
        float rgba_blue[4] = {0.3f, 0.6f, 1.0f, 1.0f}; // blue
        float rgba_red[4] = {1.0f, 0.3f, 0.2f, 1.0f};  // red

        // 画出deformable object sdf
        for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
        {
            DrawCapsule(m, scn, phyEngData_->softDef_->deformable_objects_[i].radius, i + 200, phyEngData_->human_sdf_[i], rgba_red);
        }

        // 画出obstracle object sdf
        for (int i = 0; i < phyEngData_->softDef_->num_obstracles_; ++i)
        {
            DrawCapsule(m, scn, phyEngData_->softDef_->obstracle_objects_[i].radius, i + 100, phyEngData_->robot_sdf_[i], rgba_blue);
        }

        // 更新节点位置
        for (size_t i = 0; i < phyEngData_->tet_obj_.size(); ++i)
        {
            for (size_t j = 0; j < phyEngData_->tet_obj_[i]->cur_pos.size(); ++j)
            {
                cuVec3 pos = phyEngData_->tet_obj_[i]->cur_pos[j];
                pos = phyEngData_->tet_obj_[i]->GetGlobalPos(pos);
                phyEngData_->vertices_pos_[i][j][0] = pos.x;
                phyEngData_->vertices_pos_[i][j][1] = pos.y;
                phyEngData_->vertices_pos_[i][j][2] = pos.z;
            }
        }

        scn->maxgeom = phyEngData_->softDef_->max_geom_;
        // 画出mesh
        for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
        {
            for (size_t j = 0; j < phyEngData_->edges_[i].size(); ++j)
            {
                if (scn->ngeom + 3 > scn->maxgeom)
                {
                    mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);
                    return;
                }
                const int v0 = phyEngData_->edges_[i][j][0];
                const int v1 = phyEngData_->edges_[i][j][1];
                mjvGeom *geom = &scn->geoms[scn->ngeom++];
                mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);
                geom->category = mjCAT_DECOR;
                geom->objtype = mjOBJ_UNKNOWN;
                const mjtNum from[3] = {phyEngData_->vertices_pos_[i][v0][0], phyEngData_->vertices_pos_[i][v0][1], phyEngData_->vertices_pos_[i][v0][2]};
                const mjtNum to[3] = {phyEngData_->vertices_pos_[i][v1][0], phyEngData_->vertices_pos_[i][v1][1], phyEngData_->vertices_pos_[i][v1][2]};
                mjv_connector(geom, mjGEOM_LINE, 0.001, from, to);
            }
        }

        // 接触力可视化
        if (phyEngData_->softDef_->show_contact_force_)
        {
            for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
            {
                for (int j = 0; j < phyEngData_->softDef_->num_obstracles_; ++j)
                {
                    auto forces = phyEngData_->contact_forces_[i][j].contact_forces;
                    auto points = phyEngData_->contact_forces_[i][j].contact_points;

                    for (size_t k = 0; k < points.size(); ++k)
                    {
                        mjvGeom *geom = &scn->geoms[scn->ngeom++];
                        mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_blue);
                        geom->category = mjCAT_DECOR;
                        geom->objtype = mjOBJ_UNKNOWN;

                        auto pos = points[k];
                        auto f = forces[k] * phyEngData_->softDef_->force_vis_scale_;

                        const mjtNum from[3] = {pos.x, pos.y, pos.z};
                        const mjtNum to[3] = {pos.x - f.x, pos.y - f.y, pos.z - f.z};
                        mjv_connector(geom, mjGEOM_LINE, 2.0, from, to);
                    }
                }
            }
        }

        if (phyEngData_->softDef_->show_total_contact_force_)
        {
            for (int i = 0; i < phyEngData_->softDef_->num_deformable_objects_; ++i)
            {
                for (int j = 0; j < phyEngData_->softDef_->num_obstracles_; ++j)
                {
                    {
                        mjvGeom *geom = &scn->geoms[scn->ngeom++];
                        mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_red);
                        geom->category = mjCAT_DECOR;
                        geom->objtype = mjOBJ_UNKNOWN;

                        auto pos = phyEngData_->contact_forces_[i][j].total_contact_force_pos;
                        auto f = phyEngData_->contact_forces_[i][j].total_contact_force * phyEngData_->softDef_->force_vis_scale_;

                        const mjtNum from[3] = {pos.x, pos.y, pos.z};
                        const mjtNum to[3] = {pos.x - f.x, pos.y - f.y, pos.z - f.z};
                        mjv_connector(geom, mjGEOM_LINE, 5.0, from, to);
                    }

                    {
                        mjvGeom *geom = &scn->geoms[scn->ngeom++];
                        mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_blue);
                        geom->category = mjCAT_DECOR;
                        geom->objtype = mjOBJ_UNKNOWN;

                        auto pos = phyEngData_->contact_forces_[i][j].total_contact_force_pos;
                        auto f = phyEngData_->contact_forces_[i][j].total_contact_torque * phyEngData_->softDef_->force_vis_scale_;

                        const mjtNum from[3] = {pos.x, pos.y, pos.z};
                        const mjtNum to[3] = {pos.x - f.x, pos.y - f.y, pos.z - f.z};
                        mjv_connector(geom, mjGEOM_LINE, 5.0, from, to);
                    }
                }
            }
        }
    }

    void HumanRobotPhysicalContact::DrawCapsule(const mjModel *m, mjvScene *scn, mjtNum radius, int lable_id, std::shared_ptr<SDFCapsule> sdf, float *rgba)
    {
        // mjvGeom for obstracle display
        cuVec3 pp1 = sdf->a;
        cuVec3 pp2 = sdf->b;
        mjtNum pos_1[3] = {pp1.x, pp1.y, pp1.z};
        mjtNum pos_2[3] = {pp2.x, pp2.y, pp2.z};
        mjvGeom *capsule = AddCapsuleToScene(pos_1, pos_2, radius, rgba, scn);
        if (phyEngData_->softDef_->show_sdf_label_)
        {
            makeLabelX(m, mjOBJ_GEOM, lable_id, capsule->label);
        }
    }

    void HumanRobotPhysicalContact::InitSim()
    {
        // std::cout << "Init.." << std::endl;
    }

    void HumanRobotPhysicalContact::StartSim()
    {
        // std::cout << "  StartSim" << std::endl;
    }

    void HumanRobotPhysicalContact::StepSim()
    {
        // std::cout << "  step.." << std::endl;
    }

    void HumanRobotPhysicalContact::SubStepSim()
    {
        // std::cout << "    substep.." << std::endl;
    }

} // namespace mujoco::plugin::simplysoft