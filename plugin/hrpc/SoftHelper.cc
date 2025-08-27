#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>

#include "SoftHelper.hh"
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::hrpc
{
    mjvGeom *AddSphereToScene(const mjtNum pos[3], mjtNum radius, const float rgba[4], mjvScene *scn)
    {
        // 如果几何数量超出限制
        if (scn->ngeom >= scn->maxgeom)
        {
            mj_warning(NULL, mjWARN_VGEOMFULL, scn->maxgeom);
            return NULL;
        }

        // 获取一个空闲几何引用
        mjvGeom *thisgeom = scn->geoms + scn->ngeom;

        // 设置大小（mjvGeom 接受的是 mjtNum 指针）
        mjtNum size[3] = {radius, 0, 0}; // 只有第一个分量有效
        mjv_initGeom(thisgeom, mjGEOM_SPHERE, size, pos, NULL, rgba);

        // 可选：设置对象 ID 等（非必须）
        thisgeom->objtype = mjOBJ_UNKNOWN;
        thisgeom->objid = -1;
        thisgeom->category = mjCAT_DECOR; // 用作装饰而不干扰仿真对象

        // 增加计数
        scn->ngeom++;

        return thisgeom;
    }

    mjvGeom *AddCapsuleToScene(const mjtNum start[3], const mjtNum end[3], mjtNum radius, const float rgba[4], mjvScene *scn)
    {
        // 检查几何缓冲区容量
        if (scn->ngeom >= scn->maxgeom)
        {
            mj_warning(nullptr, mjWARN_VGEOMFULL, scn->maxgeom);
            return nullptr;
        }

        // 获取当前几何指针
        mjvGeom *thisgeom = scn->geoms + scn->ngeom;

        // 计算胶囊参数
        mjtNum center[3]; // 胶囊中心点
        mjtNum axis[3];   // 起点到终点的方向向量
        mjtNum height;    // 两点间距离

        // 计算中心点（网页3的坐标变换思想）
        mju_add3(center, start, end);
        mju_scl3(center, center, 0.5);

        // 计算方向向量和高度（网页6的几何计算逻辑）
        mju_sub3(axis, end, start);
        height = mju_normalize3(axis);

        // 设置胶囊尺寸参数（MuJoCo规范[5](@ref)）
        mjtNum size[3] = {radius, height / 2, 0}; // [半径, 半高, 保留参数]

        // 生成旋转矩阵（基于网页3的四元数旋转方法）
        mjtNum z_axis[3] = {0, 0, 1};         // 默认Z轴方向
        mjtNum z_axis_default[3] = {0, 1, 0}; // 默认Z轴方向
        mjtNum cross[3], quat[4], mat[9];

        // 计算旋转轴和角度
        mju_cross(cross, z_axis, axis);
        mjtNum angle = mju_acos(mju_dot3(z_axis, axis));
        mjtNum cross_norm = mju_normalize3(cross);

        // 处理临界情况
        const mjtNum EPSILON = 1e-6;
        if (cross_norm < EPSILON)
        {
            mju_cross(cross, z_axis_default, axis);
            angle = mju_acos(mju_dot3(z_axis_default, axis));
        }

        // 生成旋转四元数
        mju_axisAngle2Quat(quat, cross, angle);
        mju_quat2Mat(mat, quat);

        // 初始化几何体（遵循MuJoCO的mjv_initGeom规范[5](@ref)）
        mjv_initGeom(thisgeom, mjGEOM_CAPSULE, size, center, mat, rgba);

        // 设置对象属性
        thisgeom->objtype = mjOBJ_UNKNOWN;
        thisgeom->objid = -1;
        thisgeom->category = mjCAT_DECOR;
        thisgeom->segid = -1;

        // 更新场景几何计数
        scn->ngeom++;

        return thisgeom;
    }

    // make text label
    void makeLabelX(const mjModel *m, mjtObj type, int id, char *label)
    {
        const char *typestr = mju_type2Str(type);
        const char *namestr = mj_id2name(m, type, id);
        char txt[100];

        // copy existing name or make numeric name
        if (namestr)
        {
            snprintf(txt, 20, "%s", namestr);
        }
        else if (typestr)
        {
            snprintf(txt, 20, "%s %d", typestr, id);
        }
        else
        {
            snprintf(txt, 20, "%d", id);
        }

        // copy result into label
        strncpy(label, txt, 100);
        label[99] = '\0';
    }
}