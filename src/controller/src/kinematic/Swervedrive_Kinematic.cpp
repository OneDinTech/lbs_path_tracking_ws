#include "Swervedrive_Kinematic.hpp"

// wrap an angle DELTA to (-pi, pi]. Only used for computing the
// shortest-path error between two absolute angles -- never applied to
// the running phi[i] itself, which stays continuous/unwrapped.
static float wrap_delta(float a)
{
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// ═══════════════════════════ 4-wheel ═══════════════════════════

void swerve_init(Swerve_Robot *sr, float half_W, float half_L, float r)
{
    sr->lx[0] =  half_W;
    sr->lx[1] = -half_W;
    sr->lx[2] = -half_W;
    sr->lx[3] =  half_W;

    sr->ly[0] =  half_L;
    sr->ly[1] =  half_L;
    sr->ly[2] = -half_L;
    sr->ly[3] = -half_L;

    sr->r = r;
}

void robot_control(Swerve_Robot *sr, float *wheel_speed, float *phi,
                   float v_bx, float v_by, float omega_bz)
{
    for (uint8_t i = 0; i < NUM_MODULES; i++)
    {
        float vx = v_bx + omega_bz * sr->lx[i];
        float vy = v_by + omega_bz * sr->ly[i];

        float speed  = std::sqrt(vx*vx + vy*vy) / sr->r;
        float target = std::atan2(vy, vx);

        float err = target - phi[i];
        while (err >  M_PI) err -= 2.0f * M_PI;
        while (err < -M_PI) err += 2.0f * M_PI;
        if (std::fabs(err) > M_PI / 2.0f)
        {
            target += (target > 0 ? -M_PI : M_PI);
            speed   = -speed;
        }
        wheel_speed[i] = speed;
        phi[i]         = target;
    }
}

void get_robot_odom_pose(Swerve_Robot *sr, float *wheel_speed, float *phi,
                         float Ts, float *x, float *y, float *theta)
{
    float v_bx = 0.0f, v_by = 0.0f, diff = 0.0f, l = 0.0f;

    for (uint8_t i = 0; i < NUM_MODULES; i++)
    {
        float vi_x = wheel_speed[i] * std::cos(phi[i]) * sr->r;
        float vi_y = wheel_speed[i] * std::sin(phi[i]) * sr->r;

        v_bx += vi_x / 4.0f;
        v_by += vi_y / 4.0f;

        diff += vi_x * sr->lx[i] + vi_y * sr->ly[i];
        l    += (sr->lx[i] * sr->lx[i]) + (sr->ly[i] * sr->ly[i]);
    }

    *x     += Ts * (std::cos(*theta) * v_bx - std::sin(*theta) * v_by);
    *y     += Ts * (std::sin(*theta) * v_bx + std::cos(*theta) * v_by);
    *theta += Ts * diff / l;
}

// ═══════════════════════════ 3-wheel ═══════════════════════════
// Matches Eq 2.1-2.5 exactly: Vi = V + Vir, expanded per-wheel as
//   wheel1: v1x=vx,              v1y=vy+w*l
//   wheel2: v2x=vx-w*l*sin(60),  v2y=vy-w*l*sin(30)
//   wheel3: v3x=vx+w*l*sin(60),  v3y=vy-w*l*sin(30)
//   v_i = sqrt(vix^2+viy^2), theta_i = atan2(viy, vix)
// via the general form vx_i = vx - w*ly[i], vy_i = vy + w*lx[i]
// (code wheel0/1/2 = reference wheel1/2/3, see swerve_init3 below).

void swerve_init3(Swerve_Robot3 *sr, float l, float r)
{
    // wheel 0: on the +X axis            -> (lx, ly) = (l, 0)
    // wheel 1: +120 deg from wheel 0     -> (-l*sin(30deg), +l*sin(60deg))
    // wheel 2: -120 deg from wheel 0     -> (-l*sin(30deg), -l*sin(60deg))
    sr->lx[0] =  l;
    sr->ly[0] =  0.0f;

    sr->lx[1] = -l * std::sin(float(M_PI) / 6.0f);
    sr->ly[1] =  l * std::sin(float(M_PI) / 3.0f);

    sr->lx[2] = -l * std::sin(float(M_PI) / 6.0f);
    sr->ly[2] = -l * std::sin(float(M_PI) / 3.0f);

    sr->l = l;
    sr->r = r;
}

void robot_control3(Swerve_Robot3 *sr, float *wheel_speed, float *phi,
                    float v_bx, float v_by, float omega_bz)
{
    // NOTE: no internal negation of omega_bz here (removed).
    // A previous version flipped omega_bz's sign here, justified by
    // "get_robot_odom_pose3 derives omega purely from the resulting
    // wheel_speed/phi, so it doesn't break IK/FK self-consistency."
    // That's true ONLY when get_robot_odom_pose3 is actually closing
    // the loop (e.g. simulation) -- FK just reads back whatever
    // convention IK used, so an extra internal flip is invisible
    // there either way.
    //
    // path_tracker.cpp does NOT use get_robot_odom_pose3 -- theta_
    // comes from a real gyro (/imu/data_raw), integrated
    // independently of this function. Against that independent
    // ground truth, the negation was a real sign flip, and it
    // stacked with the (correct, candump-validated) per-wheel
    // wheel_speed negation in path_tracker.cpp's tx_step -- for
    // pure/dominant rotation the two compounded instead of
    // canceling, so a positive commanded omega produced a
    // physically negative (opposite) yaw rate. That turned the true
    // heading target into an unstable equilibrium and its antipode
    // into a stable trap, which is what caused the CW/CCW hunting
    // oscillation at wp2 instead of convergence.
    //
    // If this function is ever reused in a context where
    // get_robot_odom_pose3 DOES close the loop, that's still fine
    // without the negation -- self-consistency there doesn't depend
    // on this sign either way. Only re-add a negation here if you
    // empirically confirm (e.g. via a stationary rotate + gyro
    // check) that THIS specific hardware chain needs it -- don't
    // reintroduce it speculatively.

    for (uint8_t i = 0; i < NUM_MODULES_3; i++)
    {
        // standard tangential rotation term: omega x r = (-omega*ly, omega*lx)
        float vx = v_bx - omega_bz * sr->ly[i];
        float vy = v_by + omega_bz * sr->lx[i];

        float v_mag        = std::sqrt(vx*vx + vy*vy);
        float target_speed = v_mag / sr->r;

        // FIX 4: zero-twist guard. Without this, a zero (or near zero)
        // commanded twist makes atan2(0,0) return 0 and every wheel
        // physically re-steers to angle 0 each time the robot stops
        // (after all_done_, with centered sticks in manual mode, at leg
        // transitions). Below 1 cm/s of wheel-ground speed: stop the
        // drive motor and HOLD the current steering angle instead of
        // re-steering on numerical noise.
        if (v_mag < 0.01f) {
            wheel_speed[i] = 0.0f;
            continue;   // phi[i] deliberately left unchanged
        }

        float target_angle = std::atan2(vy, vx);   // standard: 0 = +X = forward

        // two candidate absolute angles 180 deg apart; pick whichever
        // needs the smaller steering move from the current (continuous,
        // possibly multi-turn) phi[i]
        float theta1 = target_angle;
        float theta2 = target_angle - float(M_PI);

        float error1 = wrap_delta(theta1 - phi[i]);
        float error2 = wrap_delta(theta2 - phi[i]);

        if (std::fabs(error1) <= float(M_PI) / 2.0f)
        {
            phi[i]         = phi[i] + error1;   // continuous, not wrapped
            wheel_speed[i] = target_speed;
        }
        else
        {
            phi[i]         = phi[i] + error2;   // continuous, not wrapped
            wheel_speed[i] = -target_speed;
        }
    }
}

void get_robot_odom_pose3(Swerve_Robot3 *sr, float *wheel_speed, float *phi,
                          float Ts, float *x, float *y, float *theta)
{
    float v_bx = 0.0f, v_by = 0.0f, diff = 0.0f, l = 0.0f;

    for (uint8_t i = 0; i < NUM_MODULES_3; i++)
    {
        // standard: vi_x = speed*cos(phi), vi_y = speed*sin(phi)
        float vi_x = wheel_speed[i] * std::cos(phi[i]) * sr->r;
        float vi_y = wheel_speed[i] * std::sin(phi[i]) * sr->r;

        v_bx += vi_x / 3.0f;
        v_by += vi_y / 3.0f;

        // omega recovery consistent with vx=Vx-omega*ly, vy=Vy+omega*lx:
        // diff = sum(vi_y*lx - vi_x*ly), matches the canonical
        // ros2_control formula omega = sum(viy*lx - vix*ly) / sum(lx^2+ly^2)
        diff += vi_y * sr->lx[i] - vi_x * sr->ly[i];
        l    += (sr->lx[i] * sr->lx[i]) + (sr->ly[i] * sr->ly[i]);
    }

    *x     += Ts * (std::cos(*theta) * v_bx - std::sin(*theta) * v_by);
    *y     += Ts * (std::sin(*theta) * v_bx + std::cos(*theta) * v_by);
    *theta += Ts * diff / l;
}