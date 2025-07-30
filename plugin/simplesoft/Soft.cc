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

#include <iostream>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>

#include "Soft.hh"
#include "SoftHelper.hh"

namespace mujoco::plugin::simplysoft
{

    // Jet color palette
    void scalar2rgba(float rgba[4], mjtNum stress[3], mjtNum vmin, mjtNum vmax)
    {
        // L2 norm of the stress
        mjtNum v = mju_norm3(stress);
        v = v < vmin ? vmin : v;
        v = v > vmax ? vmax : v;
        mjtNum dv = vmax - vmin;

        if (v < (vmin + 0.25 * dv))
        {
            rgba[0] = 0;
            rgba[1] = 4 * (v - vmin) / dv;
            rgba[2] = 1;
        }
        else if (v < (vmin + 0.5 * dv))
        {
            rgba[0] = 0;
            rgba[1] = 1;
            rgba[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
        }
        else if (v < (vmin + 0.75 * dv))
        {
            rgba[0] = 4 * (v - vmin - 0.5 * dv) / dv;
            rgba[1] = 1;
            rgba[2] = 0;
        }
        else
        {
            rgba[0] = 1;
            rgba[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
            rgba[2] = 0;
        }
    }

    void SimpleSoft::RegisterPlugin()
    {
        mjpPlugin plugin;
        mjp_defaultPlugin(&plugin);

        plugin.name = "mujoco.simplysoft";
        plugin.capabilityflags |= mjPLUGIN_PASSIVE;
        // plugin.capabilityflags |= mjPLUGIN_SENSOR; // my

        // 增加模型相关配置参数
        // const char* attributes[] = {"mov", "model", "scale", "pos"};
        const char *attributes[] = {"model", "scale", "wireframe", "color_max", "sdf_radius", "sim_step", "barrier_valid", "force_feedback"};
        plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
        plugin.attributes = attributes;
        plugin.nstate = +[](const mjModel *m, int instance)
        { return 214 * 3 * 2; }; // return force number
        // plugin.nsensordata =  // my

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

    std::optional<SimpleSoft> SimpleSoft::Create(const mjModel *m, mjData *d, int instance)
    {
        return SimpleSoft(m, d, instance);
    }

    void SimpleSoft::BuildBindObstracles(std::shared_ptr<Phy3DEngine> phy)
    {

        glm::dquat q = glm::dquat(init_rot); // eulur to quaternion.
        glm::dmat3 R = glm::toMat3(q);       // to mat

        cuVec3 l1 = (R * l1pos) * jt_scale + init_offset;
        cuVec3 l2 = (R * l2pos) * jt_scale + init_offset;
        cuVec3 r1 = (R * r1pos) * jt_scale + init_offset;
        cuVec3 r2 = (R * r2pos) * jt_scale + init_offset;

        std::cout << "l1: " << l1pos << std::endl;
        std::cout << "l2: " << l2pos << std::endl;
        std::cout << "r1: " << r1pos << std::endl;
        std::cout << "r2: " << r2pos << std::endl;

        human_obstracle_left = std::make_shared<Phy3DObstracle>();
        human_sdf_left = std::make_shared<SDFCapsule>();
        human_sdf_left->a = l1;
        human_sdf_left->b = l2;
        human_sdf_left->r = muscle_radius;
        human_obstracle_left->m_sdf = human_sdf_left;
        phy->m_phy_scene->obstracles.push_back(human_obstracle_left); // add sdf function for collision dectection.

        human_obstracle_right = std::make_shared<Phy3DObstracle>();
        human_sdf_right = std::make_shared<SDFCapsule>();
        human_sdf_right->a = r1;
        human_sdf_right->b = r2;
        human_sdf_right->r = muscle_radius;
        human_obstracle_right->m_sdf = human_sdf_right;
        phy->m_phy_scene->obstracles.push_back(human_obstracle_right); // add sdf function for collision dectection.

        if (phy->m_phy_scene->tet_objects.size() >= 2)
        {
            // find soft body for simulation.
            auto tet_left = phy->m_phy_scene->tet_objects[0];  // get two soft.
            auto tet_right = phy->m_phy_scene->tet_objects[1]; // get two soft.

            std::shared_ptr<SkiningSDFObject> skin_SDF_obj1 = std::make_shared<SkiningSDFObject>();
            skin_SDF_obj1->skin_type = SKINING_TYPE_SDF;
            skin_SDF_obj1->bind_obj = human_obstracle_left;            // bind to the right tet object.
            skin_SDF_obj1->GenerateSkinWeightsAccordingSDFs(tet_left); // generate SDF skin weights.
            tet_left->AddComponent<SkiningSDFObject>(skin_SDF_obj1);

            std::shared_ptr<SkiningSDFObject> skin_SDF_obj2 = std::make_shared<SkiningSDFObject>();
            skin_SDF_obj2->skin_type = SKINING_TYPE_SDF;
            skin_SDF_obj2->bind_obj = human_obstracle_right;            // bind to the right tet object.
            skin_SDF_obj2->GenerateSkinWeightsAccordingSDFs(tet_right); // generate SDF skin weights.
            tet_right->AddComponent<SkiningSDFObject>(skin_SDF_obj2);
        }
        else
        {
            LOG::Warning("There is no enough tet objects in scene.");
        }
    }

    SimpleSoft::SimpleSoft(const mjModel *m, mjData *d, int instance)
    {
        m_phy = std::make_shared<Phy3DEngine>();
        m_scene = std::make_shared<Phy3DScene>();
        m_phy->SetPhyScene(m_scene);
        m_phy->registerInitCallback(this, &SimpleSoft::InitSim);
        m_phy->registerStartSimulateCallback(this, &SimpleSoft::StartSim);
        m_phy->registerStepCallback(this, &SimpleSoft::StepSim);
        m_phy->registerSubStepCallback(this, &SimpleSoft::SubStepSim);

        std::string path = mj_getPluginConfig(m, instance, "model");
        scale = strtod(mj_getPluginConfig(m, instance, "scale"), nullptr);
        show_wireframe = (strtod(mj_getPluginConfig(m, instance, "wireframe"), nullptr) > 0);

        // add obstracle
        // m_radius = 0.08; // peneration
        const char *radius_str = mj_getPluginConfig(m, instance, "sdf_radius");
        m_radius = radius_str ? strtod(radius_str, nullptr) : 0.08;
        // std::cout<<m_radius<<'\n';

        robot_sdf_left = std::make_shared<SDFCapsule>();
        robot_sdf_left->r = m_radius;
        robot_obstracle_left = std::make_shared<Phy3DObstracle>();
        robot_obstracle_left->m_sdf = robot_sdf_left;
        robot_obstracle_left->m_sdf->radius = m_radius;
        robot_obstracle_left->fixed = true;
        robot_obstracle_left->m_sdf->pos = cuVec3(0.0, 0.0, 0.0); // clear position
        robot_obstracle_left->pos = robot_obstracle_left->pre_pos = robot_obstracle_left->m_sdf->pos;
        m_phy->m_phy_scene->obstracles.push_back(robot_obstracle_left);

        robot_sdf_right = std::make_shared<SDFCapsule>();
        robot_sdf_right->r = m_radius;
        robot_obstracle_right = std::make_shared<Phy3DObstracle>();
        robot_obstracle_right->m_sdf = robot_sdf_right;
        robot_obstracle_right->m_sdf->radius = m_radius;
        robot_obstracle_right->fixed = true;
        robot_obstracle_right->m_sdf->pos = cuVec3(0.0, 0.0, 0.0); // clear position
        robot_obstracle_right->pos = robot_obstracle_right->pre_pos = robot_obstracle_right->m_sdf->pos;
        m_phy->m_phy_scene->obstracles.push_back(robot_obstracle_right);

        body_id_upperarm_L_ = mj_name2id(m, mjOBJ_BODY, "upper_arm_left");
        body_id_upperarm_R_ = mj_name2id(m, mjOBJ_BODY, "upper_arm_right");
        // std::cout<<"body_id_upperarm_L_  "<<body_id_upperarm_L_<<'\n';

        // tet_obj_left  = LoadTetrahedralMesh(path, "bunny", vert_pos_left);
        // tet_obj_left  = LoadTetrahedralMesh(path, "tencent_arm", vert_pos_left);
        tet_obj_left = LoadTetrahedralMesh(path, "left_arm", vert_pos_left, surface_tris_left);
        tet_obj_right = LoadTetrahedralMesh(path, "right_arm", vert_pos_right, surface_tris_right);

        // add phy & scene.
        BuildBindObstracles(m_phy);

        const char *barrier_valid_str = mj_getPluginConfig(m, instance, "barrier_valid");
        barrier_valid = barrier_valid_str ? (strtod(barrier_valid_str, nullptr) > 0) : false;

        const char *force_feedback_str = mj_getPluginConfig(m, instance, "force_feedback");
        force_feedback = force_feedback_str ? (strtod(force_feedback_str, nullptr) > 0) : false;
    }

    std::shared_ptr<Phy3DDeformableObject> SimpleSoft::LoadTetrahedralMesh(const std::string &scene_path, const std::string &config_name, std::vector<mjtNum> &vert_pos, std::vector<int> &surface_tris)
    {
        m_phy->config->scene_folder = scene_path;
        m_phy->config->GetAllConfigs();

        // std::cout << m_phy->config->scene_folder << std::endl;
        // for (size_t i = 0; i < m_phy->config->yamls.size(); i++) {
        //     std::cout << i << ": " << m_phy->config->yamls[i] << std::endl;
        // }

        m_phy->config->LoadConfig(config_name);
        m_phy->config->sub_steps = 5;
        m_phy->config->collision_type = 1; // SDF collision detection
        m_phy->config->scale = 1.0;
        m_phy->config->inner_fixed = true; // set fall down
        m_phy->config->pos = init_offset;
        m_phy->config->rot = init_rot;
        m_phy->config->barrier_soft_rigid_valid = barrier_valid;

        m_phy->timer->use_real_timer = false;

        std::string tet_path = m_phy->config->GetModelPath();
        auto tet_obj = std::make_shared<Phy3DDeformableObject>();
        tet_obj->LoadFromFile(tet_path);

        if (m_phy->config->sampled_points_path.size() > 0)
        {
            tet_obj->LoadSoftSensorPointsFromFile(BIULAB_RESOURCES_PATH + m_phy->config->sampled_points_path);
        }

        // TODO: 这里的变换有问题, 因为仿真使用不是全局的位置去仿真的，所有希望直接设置位置sph->pos去移动物体是不起作用的，希望后边换成全局的位置去进行仿真的计算。
        if (m_phy->config->apply_transform)
        {
            std::vector<Vertex> vertices = {};
            tet_obj->MoveAndScaleToCenter(m_phy->config->scale, m_phy->config->rot, m_phy->config->pos, vertices, m_phy->config->reverse_xz, false);
        }

        tet_obj->InitPhysicsFromTetMesh(m_phy->config);

        CreateDistanceConstraints(m_phy, tet_obj, m_phy->config);
        CreateVolumeConstraints(m_phy, tet_obj, m_phy->config);
        CreateFEMVolumeConstraints(m_phy, tet_obj, m_phy->config);
        CreateRandomDistanceConstraints(m_phy, tet_obj, m_phy->config);
        CreateDirectConstraints(m_phy, tet_obj, m_phy->config);
        std::shared_ptr<Phy3DObstracle> RigidBody = m_phy->m_phy_scene->obstracles[barrier_id];
        barrier_id++;
        // std::cout<<m_phy->m_phy_scene->obstracles.size()<<' '<<barrier_id<<'\n';
        CreateBarrierConstraints(m_phy, tet_obj, RigidBody, m_phy->config);

        m_phy->m_phy_scene->AddTetMesh(tet_obj);
        m_phy->InitEngine();

        for (size_t i = 0; i < tet_obj->tetSurfaceTriIds.size(); i++)
        {
            auto ret = tet_obj->tetSurfaceTriIds[i].data;
            surface_tris.push_back(ret.x); // only record triangle vertex index.
            surface_tris.push_back(ret.y); // only record triangle vertex index.
            surface_tris.push_back(ret.z); // only record triangle vertex index.
        }

        vert_pos.resize(3 * tet_obj->verts.size());
        for (size_t i = 0; i < tet_obj->verts.size(); ++i)
        {
            cuVec3 pos = tet_obj->verts[i].data;
            pos = tet_obj->GetGlobalPos(pos);
            UniX2MuJoCO(pos);
            vert_pos[3 * i] = pos.x;
            vert_pos[3 * i + 1] = pos.y;
            vert_pos[3 * i + 2] = pos.z;
        }

        return tet_obj;
    }

    void SimpleSoft::Reset(const mjModel *m, int instance) {}

    void SimpleSoft::Compute(const mjModel *m, mjData *d, int instance)
    {
        static int compute_counter = -1;
        compute_counter++;
        // update robot and human's joint position.
        if (instance && m && d)
        {
            mj_forward(m, d);
        }
        const char *sim_step_str = mj_getPluginConfig(m, instance, "sim_step");
        sim_step = sim_step_str ? strtod(sim_step_str, nullptr) : 5;
        if (compute_counter % sim_step != 0)
            return;
        // std::cout << "[Compute] Called " << compute_counter << " times.\n";
        // static cppt::Timer t;
        // t.StopMS();

        // m_phy->SimlateSomeSteps(1);
        // cuType dt = m_phy->GetStepTime(); // doesn't work use real timer.
        // add force to arm
        if (force_feedback == true)
        {
            cuVec3 Fmj = total_force_left;
            cuVec3 Pmj = centroid_left;
            UniX2MuJoCO(Fmj);
            UniX2MuJoCO(Pmj);
            applyForceToBodyChain(m, d, body_id_upperarm_L_, Fmj, Pmj);

            Fmj = total_force_right;
            Pmj = centroid_right;
            UniX2MuJoCO(Fmj);
            UniX2MuJoCO(Pmj);
            applyForceToBodyChain(m, d, body_id_upperarm_R_, Fmj, Pmj);
        }

        m_phy->config->step_one = true;
        cuType dt = 0.016 * sim_step;
        m_phy->SimlateStep(dt);

        for (size_t i = 0; i < tet_obj_left->cur_pos.size(); ++i)
        {
            cuVec3 pos = tet_obj_left->cur_pos[i];
            pos = tet_obj_left->GetGlobalPos(pos);
            UniX2MuJoCO(pos);
            vert_pos_left[3 * i] = pos.x;
            vert_pos_left[3 * i + 1] = pos.y;
            vert_pos_left[3 * i + 2] = pos.z;
        }
        for (size_t i = 0; i < tet_obj_left->cur_pos.size(); ++i)
        {
            cuVec3 pos = tet_obj_left->cur_pos[i];
            pos = tet_obj_left->GetGlobalPos(pos);
            UniX2MuJoCO(pos);
            vert_pos_right[3 * i] = pos.x;
            vert_pos_right[3 * i + 1] = pos.y;
            vert_pos_right[3 * i + 2] = pos.z;
        }

        auto GetJointPos = [&](const char *name)
        {
            int joint_id = mj_name2id(m, mjOBJ_JOINT, name);
            int body_id = m->jnt_bodyid[joint_id];
            return d->xpos + 3 * body_id;
        };
        auto GetBodyPos = [&](const char *name)
        {
            int body_id = mj_name2id(m, mjOBJ_BODY, name);
            return d->xpos + 3 * body_id;
        };

        // update human's soft position.
        if (human_sdf_left && human_sdf_right)
        {
            human_pos_left1 = GetJointPos("shoulder2_right"); // [human] d->xanchor + 17 * 3;   // joint 17
            human_pos_left2 = GetJointPos("elbow_right");     // [human] d->xanchor + 18 * 3;   // joint 18
            human_pos_left3 = GetBodyPos("hand_right");       // [human] d->geom_xpos + 16 * 3; // ngeo 16
            human_pos_right1 = GetJointPos("shoulder2_left"); // [human] d->xanchor + 20 * 3;   // joint 20
            human_pos_right2 = GetJointPos("elbow_left");     // [human] d->xanchor + 21 * 3;   // joint 21
            human_pos_right3 = GetBodyPos("hand_left");       // [human] d->geom_xpos + 19 * 3; // ngeo 19
            cuVec3 pos1 = cuVec3(human_pos_left1[0], human_pos_left1[1], human_pos_left1[2]);
            cuVec3 pos2 = cuVec3(human_pos_left2[0], human_pos_left2[1], human_pos_left2[2]);
            cuVec3 pos3 = cuVec3(human_pos_right1[0], human_pos_right1[1], human_pos_right1[2]);
            cuVec3 pos4 = cuVec3(human_pos_right2[0], human_pos_right2[1], human_pos_right2[2]);
            UniX2MuJoCO(pos1);
            UniX2MuJoCO(pos2);
            UniX2MuJoCO(pos3);
            UniX2MuJoCO(pos4);

            human_sdf_left->a = pos1;  // set position,
            human_sdf_left->b = pos2;  // set position,
            human_sdf_right->a = pos3; // set position,
            human_sdf_right->b = pos4; // set position,
        }

        // update robot's soft position.
        if (robot_sdf_left && robot_sdf_right)
        {
            robot_pos_left1 = GetJointPos("J_SHOULDER_L2"); // [robot]
            robot_pos_left2 = GetJointPos("J_ELBOW_L1");    // [robot]
            robot_pos_left3 = GetJointPos("J_WRIST_L1");    // [robot]
            robot_pos_left4 = GetJointPos("J_F0_L0");       // [robot]
            robot_pos_left5 = GetJointPos("J_F2_L0");       // [robot]

            robot_pos_right1 = GetJointPos("J_SHOULDER_R2"); // [robot]
            robot_pos_right2 = GetJointPos("J_ELBOW_R1");    // [robot]
            robot_pos_right3 = GetJointPos("J_WRIST_R1");    // [robot]
            robot_pos_right4 = GetJointPos("J_F0_R0");       // [robot]
            robot_pos_right5 = GetJointPos("J_F2_R0");       // [robot]
            cuVec3 pos1 = cuVec3(robot_pos_left4[0], robot_pos_left4[1], robot_pos_left4[2]);
            cuVec3 pos2 = cuVec3(robot_pos_left5[0], robot_pos_left5[1], robot_pos_left5[2]);
            cuVec3 pos3 = cuVec3(robot_pos_right4[0], robot_pos_right4[1], robot_pos_right4[2]);
            cuVec3 pos4 = cuVec3(robot_pos_right5[0], robot_pos_right5[1], robot_pos_right5[2]);
            UniX2MuJoCO(pos1);
            UniX2MuJoCO(pos2);
            UniX2MuJoCO(pos3);
            UniX2MuJoCO(pos4);
            robot_sdf_left->a = pos1;  // set position,
            robot_sdf_left->b = pos2;  // set position,
            robot_sdf_right->a = pos3; // set position,
            robot_sdf_right->b = pos4; // set position,
        }

        // update calculated force
        // float* state = d->plugin[instance].state;
        int stateadr = m->plugin_stateadr[instance];
        mjtNum *state = d->plugin_state + stateadr;

        total_force_left = cuVec3(0.0, 0.0, 0.0);
        centroid_left = cuVec3(0.0, 0.0, 0.0);
        total_force_right = cuVec3(0.0, 0.0, 0.0);
        centroid_right = cuVec3(0.0, 0.0, 0.0);

        for (size_t i = 0; i < lforce_num; i++)
        {
            total_force_left += tet_obj_left->sampled_force[i].f;
            centroid_left += tet_obj_left->sampled_force[i].pos;
            total_force_right += tet_obj_right->sampled_force[i].f;
            centroid_right += tet_obj_right->sampled_force[i].pos;
            state[i * 3 + 0] = tet_obj_left->sampled_force[i].f.x; // left
            state[i * 3 + 1] = tet_obj_left->sampled_force[i].f.y; // left
            state[i * 3 + 2] = tet_obj_left->sampled_force[i].f.z; // left
            // state[i * 3 + 0] = 1.0f;
            // state[i * 3 + 1] = 2.0f;
            // state[i * 3 + 2] = 3.0f;
        }
        centroid_left /= static_cast<cuType>(lforce_num);
        centroid_right /= static_cast<cuType>(rforce_num);
        // std::cout<<total_force_left<<' '<<sqrt(total_force_left.x*total_force_left.x+total_force_left.y*total_force_left.y+total_force_left.z*total_force_left.z)<<'\n';

        int state_offset = lforce_num * 3;
        for (size_t i = 0; i < rforce_num; i++)
        {
            state[state_offset + i * 3 + 0] = tet_obj_right->sampled_force[i].f.x; // right
            state[state_offset + i * 3 + 1] = tet_obj_right->sampled_force[i].f.y; // right
            state[state_offset + i * 3 + 2] = tet_obj_right->sampled_force[i].f.z; // right
            // state[state_offset + i * 3 + 0] = 12.0;
            // state[state_offset + i * 3 + 1] = 13.0;
            // state[state_offset + i * 3 + 2] = 14.0;
        }
        cuType total_norm = 0.0;
        cuType max_norm = -1.0;
        // size_t max_idx    = 0;

        for (size_t i = 0; i < tet_obj_left->sampled_force.size(); ++i)
        {
            const auto &sf = tet_obj_left->sampled_force[i];
            cuType norm = sqrt(sf.f.x * sf.f.x + sf.f.y * sf.f.y + sf.f.z * sf.f.z);
            total_norm += norm;

            if (norm > max_norm)
            {
                max_norm = norm;
                // max_idx  = i;
            }
        }

        // cuType avg_norm = total_norm / tet_obj_left->sampled_force.size();

        // const auto& sf = tet_obj_left->sampled_force[max_idx];
        // std::cout << "[L] Average force magnitude: " << avg_norm << '\n';
        // std::cout << "[L " << max_idx << "] Max force pos (" << sf.pos.x << ", "<< sf.pos.y << ", " << sf.pos.z << ") |F| " << max_norm << '\n';
    }

    void SimpleSoft::DrawCapsule(const mjModel *m, mjvScene *scn, mjtNum radius, int jid, std::shared_ptr<SDFCapsule> cap, bool is_human)
    {
        // mjvGeom for obstracle display
        float rgba_target[4] = {0.3f, 0.6f, 1.0f, 1.0f};     // blue
        float rgba_target_RED[4] = {1.0f, 0.3f, 0.2f, 1.0f}; // red
        cuVec3 pp1 = cap->a;
        cuVec3 pp2 = cap->b;
        UniX2MuJoCO(pp1);
        UniX2MuJoCO(pp2);
        mjtNum pos_1[3] = {pp1.x, pp1.y, pp1.z};
        mjtNum pos_2[3] = {pp2.x, pp2.y, pp2.z};
        if (!is_human)
        {
            mjvGeom *capsule_left = AddCapsuleToScene(pos_1, pos_2, radius, rgba_target, scn);
            makeLabelX(m, mjOBJ_GEOM, jid, capsule_left->label);
        }
        else
        {
            mjvGeom *capsule_left = AddCapsuleToScene(pos_1, pos_2, radius, rgba_target_RED, scn);
            makeLabelX(m, mjOBJ_GEOM, jid, capsule_left->label);
        }
    }

    void SimpleSoft::Visualize(const mjModel *m, mjData *d, mjvScene *scn, int instance)
    {
        // mjvGeom for robot's obstracle display
        DrawCapsule(m, scn, m_radius, 14, robot_sdf_left);
        DrawCapsule(m, scn, m_radius, 17, robot_sdf_right);

        // draw human capsule
        if (human_sdf_left && human_sdf_right)
        {
            DrawCapsule(m, scn, muscle_radius, 14, human_sdf_left, true);
            DrawCapsule(m, scn, muscle_radius, 17, human_sdf_right, true);
        }

        for (size_t i = 0; i < tet_obj_left->cur_pos.size(); ++i)
        {
            cuVec3 pos = tet_obj_left->cur_pos[i];
            pos = tet_obj_left->GetGlobalPos(pos);
            UniX2MuJoCO(pos);
            vert_pos_left[3 * i] = pos.x;
            vert_pos_left[3 * i + 1] = pos.y;
            vert_pos_left[3 * i + 2] = pos.z;
        }

        for (size_t i = 0; i < tet_obj_right->cur_pos.size(); ++i)
        {
            cuVec3 pos = tet_obj_right->cur_pos[i];
            pos = tet_obj_right->GetGlobalPos(pos);
            UniX2MuJoCO(pos);
            vert_pos_right[3 * i] = pos.x;
            vert_pos_right[3 * i + 1] = pos.y;
            vert_pos_right[3 * i + 2] = pos.z;
        }

        for (size_t i = 0; i < surface_tris_left.size(); i += 3)
        {
            if (scn->ngeom + 3 > scn->maxgeom)
            {
                mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);
                return;
            }
            const int v0 = surface_tris_left[i];
            const int v1 = surface_tris_left[i + 1];
            const int v2 = surface_tris_left[i + 2];
            const int edges[3][2] = {{v0, v1}, {v1, v2}, {v2, v0}};
            for (int e = 0; e < 3; ++e)
            {
                mjvGeom *geom = &scn->geoms[scn->ngeom++];
                mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);
                geom->category = mjCAT_DECOR;
                geom->objtype = mjOBJ_UNKNOWN;
                const mjtNum *from = vert_pos_left.data() + 3 * edges[e][0];
                const mjtNum *to = vert_pos_left.data() + 3 * edges[e][1];
                mjv_connector(geom, mjGEOM_LINE, 0.005, from, to);
            }
        }

        for (size_t i = 0; i < surface_tris_right.size(); i += 3)
        {
            if (scn->ngeom + 3 > scn->maxgeom)
            {
                mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);
                return;
            }
            const int v0 = surface_tris_right[i];
            const int v1 = surface_tris_right[i + 1];
            const int v2 = surface_tris_right[i + 2];
            const int edges[3][2] = {{v0, v1}, {v1, v2}, {v2, v0}};
            for (int e = 0; e < 3; ++e)
            {
                mjvGeom *geom = &scn->geoms[scn->ngeom++];
                mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);
                geom->category = mjCAT_DECOR;
                geom->objtype = mjOBJ_UNKNOWN;
                const mjtNum *from = vert_pos_right.data() + 3 * edges[e][0];
                const mjtNum *to = vert_pos_right.data() + 3 * edges[e][1];
                mjv_connector(geom, mjGEOM_LINE, 0.005, from, to);
            }
        }

        // render force
        float rgba_target[4] = {0.3f, 0.6f, 1.0f, 1.0f};     // blue
        float rgba_target_RED[4] = {1.0f, 0.3f, 0.2f, 1.0f}; // red
        for (size_t i = 0; i < tet_obj_left->sampled_force.size(); i++)
        {
            mjvGeom *geom = &scn->geoms[scn->ngeom++];
            mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_target);
            geom->category = mjCAT_DECOR;
            geom->objtype = mjOBJ_UNKNOWN;

            auto pos = tet_obj_left->sampled_force[i].pos;
            auto f = tet_obj_left->sampled_force[i].f * 100.0;
            UniX2MuJoCO(pos);
            UniX2MuJoCO(f);

            const mjtNum from[3] = {pos.x, pos.y, pos.z};
            const mjtNum to[3] = {pos.x + f.x, pos.y + f.y, pos.z + f.z};

            // std::cout << tet_obj_left->sampled_force[i].pos << " -> " << tet_obj_left->sampled_force[i].f << std::endl;
            mjv_connector(geom, mjGEOM_LINE, 2.0, from, to);
        }

        // ---- 画左臂合力箭头（绿色、粗） -----------------
        float rgba_green[4] = {0.f, 1.f, 0.f, 1.f};
        mjvGeom *g_L = &scn->geoms[scn->ngeom++];
        mjv_initGeom(g_L, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_green);
        g_L->category = mjCAT_DECOR;
        cuVec3 cen = centroid_left;                     // 在 Compute() 里统计得到
        cuVec3 fagg = total_force_left * cuType(100.0); // 按需缩放
        // std::cout<<"force"<<fagg<<'\n';
        UniX2MuJoCO(cen);
        UniX2MuJoCO(fagg);
        const mjtNum from_L[3] = {cen.x, cen.y, cen.z};
        const mjtNum to_L[3] = {cen.x + fagg.x, cen.y + fagg.y, cen.z + fagg.z};
        mjv_connector(g_L, mjGEOM_LINE, 10.0, from_L, to_L); // 线粗 1.0

        mjvGeom *g_R = &scn->geoms[scn->ngeom++];
        mjv_initGeom(g_R, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_green);
        g_R->category = mjCAT_DECOR;
        cen = centroid_right;                     // 在 Compute() 里统计得到
        fagg = total_force_right * cuType(100.0); // 按需缩放
        // std::cout<<"force"<<fagg<<'\n';
        UniX2MuJoCO(cen);
        UniX2MuJoCO(fagg);
        const mjtNum from_R[3] = {cen.x, cen.y, cen.z};
        const mjtNum to_R[3] = {cen.x + fagg.x, cen.y + fagg.y, cen.z + fagg.z};
        mjv_connector(g_R, mjGEOM_LINE, 10.0, from_R, to_R); // 线粗 1.0

        for (size_t i = 0; i < tet_obj_right->sampled_force.size(); i++)
        {
            mjvGeom *geom = &scn->geoms[scn->ngeom++];
            mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba_target_RED);
            geom->category = mjCAT_DECOR;
            geom->objtype = mjOBJ_UNKNOWN;

            auto pos = tet_obj_right->sampled_force[i].pos;
            auto f = tet_obj_right->sampled_force[i].f * 100.0;
            UniX2MuJoCO(pos);
            UniX2MuJoCO(f);

            const mjtNum from[3] = {pos.x, pos.y, pos.z};
            const mjtNum to[3] = {pos.x + f.x, pos.y + f.y, pos.z + f.z};

            // std::cout << tet_obj_right->sampled_force[i].pos << " -> " << tet_obj_right->sampled_force[i].f << std::endl;
            mjv_connector(geom, mjGEOM_LINE, 2.0, from, to);
        }
    }

} // namespace mujoco::plugin::simplysoft

//================ 力 → 广义力 ===================
namespace mujoco::plugin::simplysoft
{

    void SimpleSoft::applyForceToBodyChain(const mjModel *m, mjData *d,
                                           int body_id,
                                           const cuVec3 &F_world,
                                           const cuVec3 &P_world)
    {
        // (1) 刚体质心
        cuVec3 c(d->xipos[3 * body_id + 0],
                 d->xipos[3 * body_id + 1],
                 d->xipos[3 * body_id + 2]);

        // (2) τ = (P-c) × F
        cuVec3 r = P_world - c;
        cuVec3 tau(r.y * F_world.z - r.z * F_world.y,
                   r.z * F_world.x - r.x * F_world.z,
                   r.x * F_world.y - r.y * F_world.x);

        // (3) 转成 mjtNum[3]
        mjtNum force[3] = {F_world.x, F_world.y, F_world.z};
        mjtNum torque[3] = {tau.x, tau.y, tau.z};
        mjtNum point[3] = {P_world.x, P_world.y, P_world.z};

        // (4) 投射到整个关节链 → qfrc_applied
        mj_applyFT(m, d, force, torque, point, body_id, d->qfrc_applied);
    }

} // namespace mujoco::plugin::simplysoft