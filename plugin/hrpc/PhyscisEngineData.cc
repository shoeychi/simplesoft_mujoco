#include "PhyscisEngineData.hh"
#include <cuPhy/Phy3DHelper.hh>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::hrpc
{

    void ContactForces::clear()
    {
        contact_forces.clear();
        contact_points.clear();
    }

    ContactForces &ContactForces::operator=(const ContactForces &o)
    {
        this->total_contact_force = o.total_contact_force;
        this->total_contact_force_pos = o.total_contact_force_pos;
        this->total_contact_torque = o.total_contact_torque;
        this->from_tet_id = o.from_tet_id;
        this->to_sdf_id = o.to_sdf_id;
        std::copy(o.contact_forces.begin(), o.contact_forces.end(), contact_forces.begin());
        std::copy(o.contact_points.begin(), o.contact_points.end(), contact_points.begin());
        return *this;
    }

    void ContactForces::update_total_force()
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

    PhyscisEngineData::PhyscisEngineData(std::shared_ptr<SoftDef> &softDef_)
    {
        this->softDef_ = softDef_;
        // 初始化Phy3D对象
        m_phy_ = std::make_shared<Phy3DEngine>();
        m_scene_ = std::make_shared<Phy3DScene>();
        m_phy_->SetPhyScene(m_scene_);

        // 初始化obstracles
        Initializeobstracles();

        // 初始化deformable对象，并绑定sdf和obstracles
        InitializeDeformable();

        // 初始化接触力
        contact_forces_.clear();
        for (int i = 0; i < softDef_->num_deformable_objects_; ++i)
        {
            std::vector<ContactForces> cfs = {};
            for (int j = 0; j < softDef_->num_obstracles_; ++j)
            {
                ContactForces cf;
                cf.from_tet_id = i;
                cf.to_sdf_id = j;
                cfs.push_back(cf);
            }
            contact_forces_.push_back(cfs);
        }
    }

    void PhyscisEngineData::Initializeobstracles()
    {
        for (int i = 0; i < softDef_->num_obstracles_; ++i)
        {
            auto sdf = std::make_shared<SDFCapsule>();
            sdf->r = softDef_->obstracle_objects_[i].radius;

            auto obstracle = std::make_shared<Phy3DObstracle>();
            obstracle->m_sdf = sdf;
            obstracle->m_sdf->radius = softDef_->obstracle_objects_[i].radius;
            obstracle->fixed = true;
            obstracle->m_sdf->pos = cuVec3(0.0, 0.0, 0.0); // clear position
            obstracle->pos = obstracle->m_sdf->pos;
            obstracle->pre_pos = obstracle->m_sdf->pos;

            robot_sdf_.push_back(sdf);
            robot_obstracles_.push_back(obstracle);
            m_phy_->m_phy_scene->obstracles.push_back(obstracle);
        }
    }

    void PhyscisEngineData::InitializeDeformable()
    {
        // 导入deformable model
        for (int i = 0; i < softDef_->num_deformable_objects_; ++i)
        {
            std::vector<std::array<mjtNum, 3>> vertex_pos;
            std::vector<std::array<int, 3>> surface_trangle;
            std::vector<std::array<int, 2>> edge;
            auto tet_obj = LoadTetrahedralMesh(softDef_->model_path_, softDef_->deformable_objects_[0].name, vertex_pos, surface_trangle, edge);
            tet_obj_.push_back(tet_obj);
            vertices_pos_.push_back(vertex_pos);
            surface_trangles_.push_back(surface_trangle);
            edges_.push_back(edge);

            tet_rotation_ang_.push_back(0.0);
        }

        // 初始化hunman obstracles
        for (int i = 0; i < softDef_->num_deformable_objects_; ++i)
        {
            auto sdf = std::make_shared<SDFCapsule>();
            sdf->a = softDef_->deformable_objects_[i].fix_point1;
            sdf->b = softDef_->deformable_objects_[i].fix_point2;
            sdf->r = softDef_->deformable_objects_[i].radius;
            sdf->ang = 0.0;
            auto obstracle = std::make_shared<Phy3DObstracle>();
            obstracle->m_sdf = sdf;

            human_sdf_.push_back(sdf);
            human_obstracles_.push_back(obstracle);
            m_phy_->m_phy_scene->obstracles.push_back(obstracle); // add sdf function for collision dectection.
        }

        // human sdf和deformable绑定
        for (int i = 0; i < softDef_->num_deformable_objects_; ++i)
        {
            auto tet_obj = m_phy_->m_phy_scene->tet_objects[i];
            auto skin_SDF_obj = std::make_shared<SkiningSDFObject>();
            skin_SDF_obj->skin_type = SKINING_TYPE_SDF;
            skin_SDF_obj->bind_obj = human_obstracles_[i];
            skin_SDF_obj->GenerateSkinWeightsAccordingSDFs(tet_obj);
            tet_obj->AddComponent<SkiningSDFObject>(skin_SDF_obj);
        }
    }

    std::shared_ptr<Phy3DDeformableObject> PhyscisEngineData::LoadTetrahedralMesh(const std::string &scene_path, const std::string &config_name, std::vector<std::array<mjtNum, 3>> &vert_pos, std::vector<std::array<int, 3>> &surface_tris, std::vector<std::array<int, 2>> &edges)
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
        m_phy_->config->barrier_soft_rigid_valid = softDef_->barrier_valid_;
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

        for (int i = 0; i < softDef_->num_obstracles_; ++i)
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
            std::array<mjtNum, 3> v_pos = {pos.x, pos.y, pos.z};
            vert_pos.push_back(v_pos);
        }

        edges.clear();
        for (size_t i = 0; i < surface_tris.size(); ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                int n = j + 1;
                if (n == 3)
                    n = 0;

                std::array<int, 2> edge = {surface_tris[i][j], surface_tris[i][n]};
                if (edge[0] > edge[1])
                    edge = {surface_tris[i][n], surface_tris[i][j]};

                bool has_edge = false;
                for (size_t k = 0; k < edges.size(); ++k)
                {
                    if (edges[k][0] == edge[0] && edges[k][1] == edge[1])
                    {
                        has_edge = true;
                        break;
                    }
                }
                if (!has_edge)
                    edges.push_back(edge);
            }
        }

        return tet_obj;
    }

}
