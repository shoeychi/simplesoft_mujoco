#pragma once

#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::simplysoft
{
    mjvGeom *AddSphereToScene(const mjtNum pos[3], mjtNum radius, const float rgba[4], mjvScene *scn);

    mjvGeom *AddCapsuleToScene(const mjtNum start[3], const mjtNum end[3], mjtNum radius, const float rgba[4], mjvScene *scn);

    // make text label
    void makeLabelX(const mjModel *m, mjtObj type, int id, char *label);
}
