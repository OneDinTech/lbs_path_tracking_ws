#ifndef SWERVEDRIVE_KINEMATIC_HPP
#define SWERVEDRIVE_KINEMATIC_HPP

#include <cmath>
#include <cstdint>

#define NUM_MODULES 4
#define NUM_MODULES_3 3

typedef struct Swerve_Robot
{
    float lx[NUM_MODULES];   // x position of each wheel from robot centre (m)
    float ly[NUM_MODULES];   // y position of each wheel from robot centre (m)
    float r;                 // wheel radius (m)
} Swerve_Robot;

// 3-wheel variant: wheel 0 on +X axis, wheels 1,2 at +-120 deg from it.
// angle convention: atan2(x, y) -- measured from +Y axis, matching the
// reference kinematics_node.py this was ported from. Steering commands
// are CONTINUOUS/unwrapped (multi-turn), not clamped to (-pi, pi].
typedef struct Swerve_Robot3
{
    float lx[NUM_MODULES_3];
    float ly[NUM_MODULES_3];
    float l;                 // distance from centre to each wheel (m)
    float r;                 // wheel radius (m)
} Swerve_Robot3;

// ─────────────────────────── 4-wheel ───────────────────────────

void swerve_init(Swerve_Robot *sr, float half_W, float half_L, float r);

void robot_control(Swerve_Robot *sr, float *wheel_speed, float *phi,
                   float v_bx, float v_by, float omega_bz);

void get_robot_odom_pose(Swerve_Robot *sr, float *wheel_speed, float *phi,
                         float Ts, float *x, float *y, float *theta);

// ─────────────────────────── 3-wheel ───────────────────────────

// l : distance from robot centre to each wheel (m)
// r : wheel radius (m)
void swerve_init3(Swerve_Robot3 *sr, float l, float r);

// inverse kinematics, ported from kinematics_node.py inverse_kinematics().
// phi[i] is the CONTINUOUS measured/commanded steering angle (radians,
// unwrapped -- can grow past +-pi for multi-turn position-mode motors).
// On return, wheel_speed[i] and phi[i] hold the new commands.
void robot_control3(Swerve_Robot3 *sr, float *wheel_speed, float *phi,
                    float v_bx, float v_by, float omega_bz);

// forward kinematics + euler odometry integration, 3-wheel layout.
// phi[i] is the same continuous angle convention as robot_control3.
void get_robot_odom_pose3(Swerve_Robot3 *sr, float *wheel_speed, float *phi,
                          float Ts, float *x, float *y, float *theta);

#endif // SWERVEDRIVE_KINEMATIC_HPP