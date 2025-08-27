#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "PhyscisEngineData.hh"

namespace mujoco::plugin::hrpc
{

  class HumanRobotPhysicalContact
  {
  public:
    HumanRobotPhysicalContact(const mjModel *m, int instance);
    ~HumanRobotPhysicalContact() = default;

    static void RegisterPlugin();

    void Reset(const mjModel *m, int instance);
    void Compute(const mjModel *m, mjData *d, int instance);
    void Visualize(const mjModel *m, mjData *d, mjvScene *scn, int instance);

  private:
    cuVec3 GetBodyPoseForSdf(const mjModel *m, const mjData *d, const std::string &body_name, const cuVec3 &pos_at_body);
    double GetRotationAngle(const cuVec3 &pos1, const cuVec3 &pos2, const cuVec3 &pos_dir1, const cuVec3 &pos_dir2);
    void DrawCapsule(const mjModel *m, mjvScene *scn, mjtNum radius, int lable_id, std::shared_ptr<SDFCapsule> sdf, float *rgba);
    bool ContactPoint2Sdf(const cuVec3 &p, const cuVec3 &a, const cuVec3 &b, double r);

    void InitSim();
    void StartSim();
    void StepSim();
    void SubStepSim();

    static PhyscisEngineData *phyEngData_;
  };

} // namespace mujoco::plugin::hrpc
