/*================================================================
*  Copyright (C)2024 All rights reserved.
*  FileName : Soft.cc
*  Author   : dlkou
*  Email    : elonkou@ktime.cc
*  Date     : Wed 23 Oct 2024 08:10:35 PM CST
================================================================*/

#include <algorithm>
#include <cstddef>
#include <optional>
#include <sstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>

#include "Soft.hh"
#include "SoftHelper.hh"

namespace mujoco::plugin::simplysoft
{

    void SimpleSoft::RegisterPlugin()
    {
        mjpPlugin plugin;
        mjp_defaultPlugin(&plugin);

        plugin.name = "mujoco.simplysoft";
        plugin.capabilityflags |= mjPLUGIN_PASSIVE;

        // 增加模型相关配置参数
        const char *attributes[] = {"config"};
        plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
        plugin.attributes = attributes;
        // plugin.nstate            = +[](const mjModel* m, int instance) { return 214 * 3 * 2; }; // return force number  ???什么作用???
        plugin.nstate = +[](const mjModel *m, int instance)
        { return 0; };

        plugin.init = +[](const mjModel *m, mjData *d, int instance)
        {
            // 无法避免初始化两次，第一次是检查
            d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new SimpleSoft(m, d, instance));
            return 0;
        };

        // 注册状态更新回调
        plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data, int instance)
        {
            auto *soft = reinterpret_cast<SimpleSoft *>(plugin_data);
            soft->Reset(m, instance);
        };

        plugin.destroy = +[](mjData *d, int instance)
        {
            delete reinterpret_cast<SimpleSoft *>(d->plugin_data[instance]);
            d->plugin_data[instance] = 0;
        };

        plugin.compute = +[](const mjModel *m, mjData *d, int instance, int capability_bit)
        {
            auto *elasticity = reinterpret_cast<SimpleSoft *>(d->plugin_data[instance]);
            elasticity->Compute(m, d, instance);
        };

        plugin.visualize = +[](const mjModel *m, mjData *d, const mjvOption *opt, mjvScene *scn,
                               int instance)
        {
            auto *elasticity = reinterpret_cast<SimpleSoft *>(d->plugin_data[instance]);
            elasticity->Visualize(m, d, scn, instance);
        };

        mjp_registerPlugin(&plugin);
    }

    SimpleSoft::SimpleSoft(const mjModel *m, mjData *d, int instance)
    {
        // 初始化Phy3D对象
        m_phy_ = std::make_shared<Phy3DEngine>();
        m_scene_ = std::make_shared<Phy3DScene>();
        m_phy_->SetPhyScene(m_scene_);
        m_phy_->registerInitCallback(this, &SimpleSoft::InitSim);
        m_phy_->registerStartSimulateCallback(this, &SimpleSoft::StartSim);
        m_phy_->registerStepCallback(this, &SimpleSoft::StepSim);
        m_phy_->registerSubStepCallback(this, &SimpleSoft::SubStepSim);

        // 读取配置文件
        std::string config_file = mj_getPluginConfig(m, instance, "config");
        LoadConfig(config_file);

        // 初始化obstracles
        Initializeobstracles();

        // 初始化deformable对象，并绑定sdf和obstracles
        InitializeDeformable();

        // 初始化接触力
        contact_forces_.clear();
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            std::vector<ContactForces> cfs = {};
            for (int j = 0; j < num_obstracles_; ++j)
            {
                ContactForces cf;
                cf.from_tet_id = i;
                cf.to_sdf_id = j;
                cfs.push_back(cf);
            }
            contact_forces_.push_back(cfs);
        }
    }

    void SimpleSoft::LoadConfig(const std::string &config_path)
    {
        // std::filesystem::path relative_path = "plugin/simplesoft/config.json";
        // std::filesystem::path path = std::filesystem::absolute(relative_path);
        std::cout << "config-file: " << config_path << std::endl;

        std::ifstream config_file(config_path);
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
            object.fix_point1 = cuVec3(p1[0], p1[1], p1[2]);
            object.fix_point2 = cuVec3(p2[0], p2[1], p2[2]);
            object.bound_body = obj_cfg["bound_body"];
            std::vector<double> p3 = obj_cfg["bound_pos1"];
            std::vector<double> p4 = obj_cfg["bound_pos2"];
            object.bound_pos1 = cuVec3(p3[0], p3[1], p3[2]);
            object.bound_pos2 = cuVec3(p4[0], p4[1], p4[2]);
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

    void SimpleSoft::Initializeobstracles()
    {
        for (int i = 0; i < num_obstracles_; ++i)
        {
            auto sdf = std::make_shared<SDFCapsule>();
            sdf->r = obstracle_objects_[i].radius;

            auto obstracle = std::make_shared<Phy3DObstracle>();
            obstracle->m_sdf = sdf;
            obstracle->m_sdf->radius = obstracle_objects_[i].radius;
            obstracle->fixed = true;
            obstracle->m_sdf->pos = cuVec3(0.0, 0.0, 0.0); // clear position
            obstracle->pos = obstracle->m_sdf->pos;
            obstracle->pre_pos = obstracle->m_sdf->pos;

            robot_sdf_.push_back(sdf);
            robot_obstracles_.push_back(obstracle);
            m_phy_->m_phy_scene->obstracles.push_back(obstracle);
        }
    }

    void SimpleSoft::InitializeDeformable()
    {
        // 导入deformable model
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            std::vector<std::array<mjtNum, 3>> vertex_pos;
            std::vector<std::array<int, 3>> surface_trangle;
            auto tet_obj = LoadTetrahedralMesh(model_path_, deformable_objects_[i].name, vertex_pos, surface_trangle);
            tet_obj_.push_back(tet_obj);
            vertices_pos_.push_back(vertex_pos);
            surface_trangles_.push_back(surface_trangle);
        }

        // 初始化hunman obstracles
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            auto sdf = std::make_shared<SDFCapsule>();
            sdf->a = deformable_objects_[i].fix_point1;
            sdf->b = deformable_objects_[i].fix_point2;
            sdf->r = deformable_objects_[i].radius;
            auto obstracle = std::make_shared<Phy3DObstracle>();
            obstracle->m_sdf = sdf;

            human_sdf_.push_back(sdf);
            human_obstracles_.push_back(obstracle);
            m_phy_->m_phy_scene->obstracles.push_back(obstracle); // add sdf function for collision dectection.
        }

        // human sdf和deformable绑定
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            auto tet_obj = m_phy_->m_phy_scene->tet_objects[i];
            auto skin_SDF_obj = std::make_shared<SkiningSDFObject>();
            skin_SDF_obj->skin_type = SKINING_TYPE_SDF;
            skin_SDF_obj->bind_obj = human_obstracles_[i];
            skin_SDF_obj->GenerateSkinWeightsAccordingSDFs(tet_obj);
            tet_obj->AddComponent<SkiningSDFObject>(skin_SDF_obj);
        }
    }

    std::shared_ptr<Phy3DDeformableObject> SimpleSoft::LoadTetrahedralMesh(const std::string &scene_path, const std::string &config_name, std::vector<std::array<mjtNum, 3>> &vert_pos, std::vector<std::array<int, 3>> &surface_tris)
    {
        m_phy_->config->scene_folder = scene_path;
        m_phy_->config->GetAllConfigs();

        m_phy_->config->LoadConfig(config_name);
        m_phy_->config->sub_steps = 5;
        m_phy_->config->collision_type = 1; // SDF collision detection
        m_phy_->config->scale = 1.0;
        m_phy_->config->inner_fixed = true; // set fall down
        m_phy_->config->pos = cuVec3(0.0, 0.0, 0.0);
        m_phy_->config->rot = glm::vec3(0.0, 0.0, 0.0);
        m_phy_->config->barrier_soft_rigid_valid = barrier_valid_;
        m_phy_->config->border.min = cuVec3(-10.0, -10.0, -10.0);
        m_phy_->config->border.max = cuVec3(10.0, 10.0, 10.0);

        m_phy_->timer->use_real_timer = false;

        std::string tet_path = m_phy_->config->GetModelPath();
        auto tet_obj = std::make_shared<Phy3DDeformableObject>();
        tet_obj->LoadFromFile(tet_path);

        // 需要获取sampled_points_path的配置文件，会获取和显示采样点的接触力
        if (m_phy_->config->sampled_points_path.size() > 0)
        {
            tet_obj->LoadSoftSensorPointsFromFile(BIULAB_RESOURCES_PATH + m_phy_->config->sampled_points_path);
        }

        // TODO: 这里的变换有问题, 因为仿真使用不是全局的位置去仿真的，所有希望直接设置位置sph->pos去移动物体是不起作用的，希望后边换成全局的位置去进行仿真的计算。
        if (m_phy_->config->apply_transform)
        {
            std::vector<Vertex> vertices = {};
            tet_obj->MoveAndScaleToCenter(m_phy_->config->scale, m_phy_->config->rot, m_phy_->config->pos, vertices, m_phy_->config->reverse_xz, false);
        }

        tet_obj->InitPhysicsFromTetMesh(m_phy_->config);

        CreateDistanceConstraints(m_phy_, tet_obj, m_phy_->config);
        CreateVolumeConstraints(m_phy_, tet_obj, m_phy_->config);
        CreateFEMVolumeConstraints(m_phy_, tet_obj, m_phy_->config);
        CreateRandomDistanceConstraints(m_phy_, tet_obj, m_phy_->config);
        CreateDirectConstraints(m_phy_, tet_obj, m_phy_->config);

        for (int i = 0; i < num_obstracles_; ++i)
        {
            std::shared_ptr<Phy3DObstracle> RigidBody = m_phy_->m_phy_scene->obstracles[i];
            CreateBarrierConstraints(m_phy_, tet_obj, RigidBody, m_phy_->config);
        }

        m_phy_->m_phy_scene->AddTetMesh(tet_obj);
        m_phy_->InitEngine();

        for (size_t i = 0; i < tet_obj->tetSurfaceTriIds.size(); ++i)
        {
            auto ret = tet_obj->tetSurfaceTriIds[i].data;
            std::array<int, 3> tri = {ret.x, ret.y, ret.z};
            surface_tris.push_back(tri); // only record triangle vertex index.
        }

        for (size_t i = 0; i < tet_obj->verts.size(); ++i)
        {
            cuVec3 pos = tet_obj->verts[i].data;
            pos = tet_obj->GetGlobalPos(pos);
            UniX2MuJoCO(pos);
            std::array<mjtNum, 3> v_pos = {pos.x, pos.y, pos.z};
            vert_pos.push_back(v_pos);
        }

        return tet_obj;
    }

    void SimpleSoft::Reset(const mjModel *m, int instance) {}

    void SimpleSoft::Compute(const mjModel *m, mjData *d, int instance)
    {
        static int compute_counter = -1;
        compute_counter++;

        // std::cout << "Call compute, step = " << compute_counter << std::endl;

        // update robot and human's joint position.
        if (instance && m && d)
        {
            mj_forward(m, d);
        }

        if (compute_counter % sim_step_ != 0)
            return;

        // 执行软体pbd仿真
        m_phy_->config->step_one = true;
        cuType dt = m->opt.timestep * sim_step_;
        m_phy_->SimlateStep(dt);

        // 更新human sdf位置
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            cuVec3 pos1 = GetBodyPoseForSdf(m, d, deformable_objects_[i].bound_body, deformable_objects_[i].bound_pos1);
            cuVec3 pos2 = GetBodyPoseForSdf(m, d, deformable_objects_[i].bound_body, deformable_objects_[i].bound_pos2);
            UniX2MuJoCO(pos1);
            UniX2MuJoCO(pos2);
            human_sdf_[i]->a = pos1; // set position,
            human_sdf_[i]->b = pos2; // set position,
        }

        // 更新robot sdf位置
        for (int i = 0; i < num_obstracles_; ++i)
        {
            cuVec3 pos1 = GetBodyPoseForSdf(m, d, obstracle_objects_[i].bound_body, obstracle_objects_[i].bound_pos1);
            cuVec3 pos2 = GetBodyPoseForSdf(m, d, obstracle_objects_[i].bound_body, obstracle_objects_[i].bound_pos2);
            UniX2MuJoCO(pos1);
            UniX2MuJoCO(pos2);
            robot_sdf_[i]->a = pos1; // set position,
            robot_sdf_[i]->b = pos2; // set position,
        }

        // 更新接触力
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            for (int j = 0; j < num_obstracles_; ++j)
            {
                contact_forces_[i][j].clear();
            }
        }
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            auto forces = m_phy_->m_phy_scene->tet_objects[i]->sampled_force;
            for (size_t j = 0; j < forces.size(); ++j)
            {
                auto pos = forces[j].pos;
                auto f = forces[j].f;
                for (int k = 0; k < num_obstracles_; ++k)
                {
                    auto a = robot_sdf_[k]->a;
                    auto b = robot_sdf_[k]->b;
                    double r = robot_sdf_[k]->r;
                    if (ContactPoint2Sdf(pos, a, b, r))
                    {
                        UniX2MuJoCO(pos);
                        UniX2MuJoCO(f);
                        contact_forces_[i][k].contact_forces.push_back(f * force_scale_);
                        contact_forces_[i][k].contact_points.push_back(pos);
                    }
                }
            }
        }
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            for (int j = 0; j < num_obstracles_; ++j)
            {
                contact_forces_[i][j].update_total_force();
            }
        }

        // 施加接触力到mujoco body上（TODO：有问题，时加的力似乎是不断叠加的）
        if (force_feedback_)
        {
            for (int i = 0; i < m->nv; ++i)
            {
                d->qfrc_applied[i] = 0.0; // 先把外力置0，否则会持续叠加，这里会不会有问题，TODO：待重力测试
            }

            for (int i = 0; i < num_deformable_objects_; ++i)
            {
                for (int j = 0; j < num_obstracles_; ++j)
                {
                    auto f = contact_forces_[i][j].total_contact_force;
                    auto t = contact_forces_[i][j].total_contact_torque;
                    auto p = contact_forces_[i][j].total_contact_force_pos;
                    int rob_body_id = mj_name2id(m, mjOBJ_BODY, obstracle_objects_[j].bound_body.c_str());
                    int hum_body_id = mj_name2id(m, mjOBJ_BODY, deformable_objects_[i].bound_body.c_str());

                    mjtNum force[3] = {f.x, f.y, f.z};
                    mjtNum torque[3] = {t.x, t.y, t.z};
                    mjtNum point[3] = {p.x, p.y, p.z};
                    mj_applyFT(m, d, force, torque, point, rob_body_id, d->qfrc_applied);

                    mjtNum force2[3] = {-f.x, -f.y, -f.z};
                    mjtNum torque2[3] = {-t.x, -t.y, -t.z};
                    mj_applyFT(m, d, force2, torque2, point, hum_body_id, d->qfrc_applied);

                    // std::cout << "force: " << f << std::endl;
                    // for (int nn = 0; nn < m->nv; nn++){
                    //     std::cout << d->qfrc_applied[nn] << " ";
                    // }
                    // std::cout << std::endl;
                }
            }
        }
    }

    cuVec3 SimpleSoft::GetBodyPoseForSdf(const mjModel *m, const mjData *d, const std::string &body_name, const cuVec3 &pos_at_body)
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

    bool SimpleSoft::ContactPoint2Sdf(const cuVec3 &p, const cuVec3 &a, const cuVec3 &b, double r)
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

    void SimpleSoft::ContactForces::update_total_force()
    {
        total_contact_force = {0.0, 0.0, 0.0};
        total_contact_force_pos = {0.0, 0.0, 0.0};
        total_contact_torque = {0.0, 0.0, 0.0};

        cuVec3 c = {0.0, 0.0, 0.0};
        for (size_t i = 0; i < contact_points.size(); ++i)
        {
            c += contact_points[i];
        }
        c.x = c.x / contact_points.size();
        c.y = c.y / contact_points.size();
        c.z = c.z / contact_points.size();

        // std::cout << "contact_points size: " << contact_points.size() << std::endl;

        mjtNum C[3] = {c.x, c.y, c.z};
        for (size_t i = 0; i < contact_points.size(); ++i)
        {
            mjtNum p[3] = {contact_points[i].x, contact_points[i].y, contact_points[i].z};
            mjtNum f[3] = {contact_forces[i].x, contact_forces[i].y, contact_forces[i].z};
            mjtNum r[3];
            mju_sub3(r, p, C);
            mjtNum tau[3];
            mju_cross(tau, r, f);

            total_contact_force.x += f[0];
            total_contact_force.y += f[1];
            total_contact_force.z += f[2];

            total_contact_torque.x += tau[0];
            total_contact_torque.y += tau[1];
            total_contact_torque.z += tau[2];
        }

        mjtNum Fc[3] = {total_contact_force.x, total_contact_force.y, total_contact_force.z};
        mjtNum Tc[3] = {total_contact_torque.x, total_contact_torque.y, total_contact_torque.z};

        // 力的方向的单位矢量
        mjtNum vec1[3];
        mju_copy3(vec1, Fc);
        mju_normalize3(vec1);

        // 扭矩的分解
        // Tc = Tc1 + Tc2， 平行和垂直于ec1
        mjtNum Tc1[3];
        mju_scl3(Tc1, vec1, mju_dot3(vec1, Tc));
        mjtNum Tc2[3];
        mju_sub3(Tc2, Tc, Tc1);

        // 找到力螺旋的位置 p2 = C + vec2，有 cross(vec2, Fc) = Tc2
        mjtNum vec2_0[3];
        mju_cross(vec2_0, Fc, Tc2);
        mjtNum vec2[3];
        mjtNum tmp = 1.0 / (mju_norm3(Fc)) / (mju_norm3(Fc));
        mju_scl3(vec2, vec2_0, tmp);
        mjtNum p2[3];
        mju_add3(p2, C, vec2);

        total_contact_force_pos = {p2[0], p2[1], p2[2]};
        total_contact_torque = {Tc1[0], Tc1[1], Tc1[2]};
    }

    void SimpleSoft::Visualize(const mjModel *m, mjData *d, mjvScene *scn, int instance)
    {
        float rgba_blue[4] = {0.3f, 0.6f, 1.0f, 1.0f}; // blue
        float rgba_red[4] = {1.0f, 0.3f, 0.2f, 1.0f};  // red

        // 画出deformable object sdf
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            DrawCapsule(m, scn, deformable_objects_[i].radius, i + 200, human_sdf_[i], rgba_red);
        }

        // 画出obstracle object sdf
        for (int i = 0; i < num_obstracles_; ++i)
        {
            DrawCapsule(m, scn, obstracle_objects_[i].radius, i + 100, robot_sdf_[i], rgba_blue);
        }

        // 更新节点位置
        for (size_t i = 0; i < tet_obj_.size(); ++i)
        {
            for (size_t j = 0; j < tet_obj_[i]->cur_pos.size(); ++j)
            {
                cuVec3 pos = tet_obj_[i]->cur_pos[j];
                pos = tet_obj_[i]->GetGlobalPos(pos);
                UniX2MuJoCO(pos);
                vertices_pos_[i][j][0] = pos.x;
                vertices_pos_[i][j][1] = pos.y;
                vertices_pos_[i][j][2] = pos.z;
            }
        }

        scn->maxgeom = max_geom_; // TODO: 去掉重复的线，减少visual geom数量。
        // 画出mesh
        for (int i = 0; i < num_deformable_objects_; ++i)
        {
            for (size_t j = 0; j < surface_trangles_[i].size(); ++j)
            {
                if (scn->ngeom + 3 > scn->maxgeom)
                {
                    mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);
                    return;
                }
                const int v0 = surface_trangles_[i][j][0];
                const int v1 = surface_trangles_[i][j][1];
                const int v2 = surface_trangles_[i][j][2];
                const int edges[3][2] = {{v0, v1}, {v1, v2}, {v2, v0}};

                for (int e = 0; e < 3; ++e)
                {
                    mjvGeom *geom = &scn->geoms[scn->ngeom++];
                    mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);
                    geom->category = mjCAT_DECOR;
                    geom->objtype = mjOBJ_UNKNOWN;
                    const mjtNum from[3] = {vertices_pos_[i][edges[e][0]][0], vertices_pos_[i][edges[e][0]][1], vertices_pos_[i][edges[e][0]][2]};
                    const mjtNum to[3] = {vertices_pos_[i][edges[e][1]][0], vertices_pos_[i][edges[e][1]][1], vertices_pos_[i][edges[e][1]][2]};
                    mjv_connector(geom, mjGEOM_LINE, 0.005, from, to);
                }
            }
        }

        // 接触力可视化（TODO: 去掉非必要节点的力，减少visual geom数量。）
        if (show_contact_force_)
        {
            for (int j = 0; j < num_deformable_objects_; ++j)
            {
                auto forces = tet_obj_[j]->sampled_force;
                for (size_t i = 0; i < forces.size(); i++)
                {
                    mjvGeom *geom = &scn->geoms[scn->ngeom++];
                    mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_blue);
                    geom->category = mjCAT_DECOR;
                    geom->objtype = mjOBJ_UNKNOWN;

                    auto pos = forces[i].pos;
                    auto f = forces[i].f * force_scale_ * force_vis_scale_;
                    UniX2MuJoCO(pos);
                    UniX2MuJoCO(f);

                    const mjtNum from[3] = {pos.x, pos.y, pos.z};
                    const mjtNum to[3] = {pos.x + f.x, pos.y + f.y, pos.z + f.z};
                    mjv_connector(geom, mjGEOM_LINE, 2.0, from, to);
                }
            }
        }

        if (show_total_contact_force_)
        {
            for (int i = 0; i < num_deformable_objects_; ++i)
            {
                for (int j = 0; j < num_obstracles_; ++j)
                {
                    {
                        mjvGeom *geom = &scn->geoms[scn->ngeom++];
                        mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_red);
                        geom->category = mjCAT_DECOR;
                        geom->objtype = mjOBJ_UNKNOWN;

                        auto pos = contact_forces_[i][j].total_contact_force_pos;
                        auto f = contact_forces_[i][j].total_contact_force * force_vis_scale_;

                        const mjtNum from[3] = {pos.x, pos.y, pos.z};
                        const mjtNum to[3] = {pos.x + f.x, pos.y + f.y, pos.z + f.z};
                        mjv_connector(geom, mjGEOM_LINE, 5.0, from, to);
                    }

                    {
                        mjvGeom *geom = &scn->geoms[scn->ngeom++];
                        mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_blue);
                        geom->category = mjCAT_DECOR;
                        geom->objtype = mjOBJ_UNKNOWN;

                        auto pos = contact_forces_[i][j].total_contact_force_pos;
                        auto f = contact_forces_[i][j].total_contact_torque * force_vis_scale_;

                        const mjtNum from[3] = {pos.x, pos.y, pos.z};
                        const mjtNum to[3] = {pos.x + f.x, pos.y + f.y, pos.z + f.z};
                        mjv_connector(geom, mjGEOM_LINE, 5.0, from, to);
                    }
                }
            }
        }
    }

    void SimpleSoft::DrawCapsule(const mjModel *m, mjvScene *scn, mjtNum radius, int lable_id, std::shared_ptr<SDFCapsule> sdf, float *rgba)
    {
        // mjvGeom for obstracle display
        cuVec3 pp1 = sdf->a;
        cuVec3 pp2 = sdf->b;
        UniX2MuJoCO(pp1);
        UniX2MuJoCO(pp2);
        mjtNum pos_1[3] = {pp1.x, pp1.y, pp1.z};
        mjtNum pos_2[3] = {pp2.x, pp2.y, pp2.z};
        mjvGeom *capsule = AddCapsuleToScene(pos_1, pos_2, radius, rgba, scn);
        if (show_sdf_label_)
        {
            makeLabelX(m, mjOBJ_GEOM, lable_id, capsule->label);
        }
    }

} // namespace mujoco::plugin::simplysoft