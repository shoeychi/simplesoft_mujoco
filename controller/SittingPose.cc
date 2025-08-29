#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <cmath>

// Constants
const double SITTING_DURATION = 2.0; // Time to achieve sitting pose (seconds)
const double MOVE_DURATION = 3.0;    // Time to move pelvis to target (seconds)
const double TOLERANCE = 0.01;       // Position tolerance (meters)
const double KP = 1000.0;            // Proportional gain for root position
const double KV = 50.0;              // Derivative gain for root position
// Constants
const double SIT_DURATION = 3.0; // Time to transition to sitting (seconds)
const double SIT_HEIGHT = 0.6;   // Target pelvis height when sitting

// Target joint angles for sitting posture (in radians)
struct SittingPose
{
    mjtNum hipFlexion = -1.57;  // Hip flexion angle
    mjtNum kneeFlexion = 1.57;  // Knee flexion angle
    mjtNum ankleFlexion = -0.3; // Ankle dorsiflexion
    mjtNum torsoLean = 0.3;     // Torso forward lean
};

// PD controller gains
struct PDGains
{
    mjtNum kp = 500.0; // Proportional gain
    mjtNum kd = 50.0;  // Derivative gain
};
// Target sitting position
const mjtNum TARGET_POS[3] = {1.3, 0.0, 0.55};

int main(int argc, char **argv)
{
    std::string mdl_path = "../../model/plugin/hrpc/sitting_pose.xml";
    if (argc > 1)
    {
        mdl_path = std::string(argv[1]);
    }
    // Load model and data
    char error[1000] = "";
    mjModel *m = nullptr;
    mjData *d = nullptr;

    // Initialize MuJoCo
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    // Load model from XML string (your provided model)
    m = mj_loadXML(mdl_path.c_str(), nullptr, error, 1000);
    if (!m)
    {
        std::cerr << "Model loading error: " << error << std::endl;
        glfwTerminate();
        return 1;
    }
    d = mj_makeData(m);

    // // Set initial root position
    // d->qpos[0] = 1.7;  // x
    // d->qpos[1] = 0.0;  // y
    // d->qpos[2] = 1.11; // z

    // Enable freejoint for root movement
    int root_body = mj_name2id(m, mjOBJ_BODY, "human_skeleton");
    if (root_body >= 0)
    {
        d->qpos[0] = 1.7;  // Initial X position
        d->qpos[1] = 0.0;  // Initial Y position
        d->qpos[2] = 1.11; // Initial Z position
    }

    // Create window and context
    GLFWwindow *window = glfwCreateWindow(1200, 900, "Human Skeleton Sitting", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Visualization objects
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    // Initialize visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(m, &scn, 1000);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Set camera to track skeleton
    cam.type = mjCAMERA_TRACKING;
    cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "human_skeleton");
    cam.distance = 3.5;

    // Define sitting pose targets
    SittingPose sitPose;
    PDGains gains;

    // Get joint IDs for critical sitting joints
    std::vector<int> hipJoints = {
        mj_name2id(m, mjOBJ_JOINT, "human_hip_L1"),
        mj_name2id(m, mjOBJ_JOINT, "human_hip_R1")};

    std::vector<int> kneeJoints = {
        mj_name2id(m, mjOBJ_JOINT, "human_knee_L"),
        mj_name2id(m, mjOBJ_JOINT, "human_knee_R")};

    std::vector<int> ankleJoints = {
        mj_name2id(m, mjOBJ_JOINT, "human_ankle_L2"),
        mj_name2id(m, mjOBJ_JOINT, "human_ankle_R2")};

    int waistJoint = mj_name2id(m, mjOBJ_JOINT, "human_waist");
    // int pelvis_id = mj_name2id(m, mjOBJ_BODY, "human_skeleton");
    bool moving_complete = false;
    // Main simulation loop
    double startTime = d->time;
    while (!glfwWindowShouldClose(window))
    {
        // Advance simulation
        mjtNum simstart = d->time;
        // double elapsed = d->time - simstart;
        // if (!moving_complete)
        // {
        //     // PID control for root position
        //     for (int i = 0; i < 3; i++)
        //     {
        //         mjtNum error = TARGET_POS[i] - d->qpos[i];
        //         mjtNum vel_error = -d->qvel[i];
        //         d->ctrl[root_body] = KP * error + KV * vel_error;
        //     }

        //     // Check if target reached
        //     mjtNum dist = 0;
        //     for (int i = 0; i < 3; i++)
        //     {
        //         dist += (TARGET_POS[i] - d->qpos[i]) * (TARGET_POS[i] - d->qpos[i]);
        //     }
        //     moving_complete = (dist < TOLERANCE * TOLERANCE) || (elapsed > SITTING_DURATION + MOVE_DURATION);
        // }

        while (d->time - simstart < 1.0 / 60.0)
        {
            // Calculate current progress in sitting motion (0-1)
            double sitProgress = std::min(1.0, (d->time - startTime) / SIT_DURATION);

            // Apply PD controllers to sitting joints
            for (int jointID : hipJoints)
            {
                if (jointID != -1)
                {
                    int qpos_adr = m->jnt_qposadr[jointID];
                    int dof_adr = m->jnt_dofadr[jointID];

                    mjtNum current = d->qpos[qpos_adr];
                    mjtNum target = sitProgress * sitPose.hipFlexion;
                    mjtNum velocity = d->qvel[dof_adr];

                    d->ctrl[jointID] = gains.kp * (target - current) - gains.kd * velocity;
                }
            }

            for (int jointID : kneeJoints)
            {
                if (jointID != -1)
                {
                    int qpos_adr = m->jnt_qposadr[jointID];
                    int dof_adr = m->jnt_dofadr[jointID];

                    mjtNum current = d->qpos[qpos_adr];
                    mjtNum target = sitProgress * sitPose.kneeFlexion;
                    mjtNum velocity = d->qvel[dof_adr];

                    d->ctrl[jointID] = gains.kp * (target - current) - gains.kd * velocity;
                }
            }

            for (int jointID : ankleJoints)
            {
                if (jointID != -1)
                {
                    int qpos_adr = m->jnt_qposadr[jointID];
                    int dof_adr = m->jnt_dofadr[jointID];

                    mjtNum current = d->qpos[qpos_adr];
                    mjtNum target = sitProgress * sitPose.ankleFlexion;
                    mjtNum velocity = d->qvel[dof_adr];

                    d->ctrl[jointID] = gains.kp * (target - current) - gains.kd * velocity;
                }
            }

            // Control torso lean
            if (waistJoint != -1)
            {
                int qpos_adr = m->jnt_qposadr[waistJoint];
                int dof_adr = m->jnt_dofadr[waistJoint];

                mjtNum current = d->qpos[qpos_adr];
                mjtNum target = sitProgress * sitPose.torsoLean;
                mjtNum velocity = d->qvel[dof_adr];

                d->ctrl[waistJoint] = gains.kp * (target - current) - gains.kd * velocity;
            }

            // Step the simulation
            mj_step(m, d);
        }

        // Update visualization
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}