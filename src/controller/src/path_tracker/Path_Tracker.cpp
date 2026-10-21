#include "Path_Tracker.hpp"

void param_init(Init *init)
{
    init->OMEGA_PREV = 0.0;
    init->V_PREV     = 0.0;
    init->VX_WORLD   = 0.0;
    init->VY_WORLD   = 0.0;
}

double get_lookahead_distance(Path_Tracker *pt, Init *init)
{
    // Adaptive look-ahead: scale with the ACTUAL current speed, not the
    // fixed desired V. init->V_PREV holds the speed commanded on the
    // previous cycle (the ramped / corner-capped value); at 100 Hz the
    // one-cycle lag is negligible. This makes the look-ahead shrink when
    // the robot slows into a corner and grow on straights, matching the
    // speed-scaled design. Fall back to pt->V before the first linear
    // velocity is computed (V_PREV == 0 at start).
    double v = (init->V_PREV > 1e-6) ? init->V_PREV : pt->V;
    double lookahead_distance = std::fabs(pt->LOOKAHEAD_TIME * v);
    return std::clamp(lookahead_distance, pt->MIN_LOOKAHEAD_DISTANCE, pt->MAX_LOOKAHEAD_DISTANCE);
}

// start_idx: skip points the robot has already passed (path pruning)
void get_lookahead_point(const double* path_x, const double* path_y,
                         const double* path_theta,
                         int n_points, int start_idx,
                         double x_V, double y_V, double L,
                         double* x_L, double* y_L, double* theta_L,
                         int* closest_idx)
{
    int idx = -1;
    for (int i = start_idx; i < n_points; i++)
    {
        double dx = path_x[i] - x_V;
        double dy = path_y[i] - y_V;
        if (std::sqrt(dx*dx + dy*dy) >= L)
        {
            idx = i; break;
        }
    }

    if (idx == -1)
    {
        *x_L         = path_x[n_points - 1];
        *y_L         = path_y[n_points - 1];
        *theta_L     = path_theta[n_points - 1];
        *closest_idx = n_points - 1;
        return;
    }

    if (idx == start_idx)
    {
        *x_L         = path_x[idx];
        *y_L         = path_y[idx];
        *theta_L     = path_theta[idx];
        *closest_idx = idx;
        return;
    }

    *closest_idx = idx - 1;

    double x1 = path_x[idx - 1] - x_V;
    double y1 = path_y[idx - 1] - y_V;
    double x2 = path_x[idx]     - x_V;
    double y2 = path_y[idx]     - y_V;

    double dx  = x2 - x1;
    double dy  = y2 - y1;
    double dr2 = dx*dx + dy*dy;

    double D   = x1*y2 - x2*y1;
    double d1  = x1*x1 + y1*y1;
    double d2  = x2*x2 + y2*y2;
    double dd  = d2 - d1;
    double sqrt_term = std::sqrt(std::max(0.0, L*L * dr2 - D*D));

    *x_L = x_V + (D*dy  + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    *y_L = y_V + (-D*dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;

    // Lookahead heading: average the two nearest path headings ON THE
    // CIRCLE. The old code converted both to [0, 2pi) then took the
    // plain arithmetic mean, which is wrong at the wrap: headings of
    // -0.02 and +0.02 became 6.26 and 0.02, averaging to ~pi instead of
    // ~0 and kicking omega for one cycle. This happens at leg starts
    // where the live heading is slightly negative. Averaging via the
    // wrapped difference takes the short way around and is correct
    // across the wrap.
    double theta1 = path_theta[idx - 1];
    double theta2 = path_theta[idx];
    double dtheta = theta2 - theta1;
    while (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    double avg = theta1 + 0.5 * dtheta;
    while (avg >  M_PI) avg -= 2.0 * M_PI;
    while (avg < -M_PI) avg += 2.0 * M_PI;
    *theta_L = avg;
}

// straight-line distance from the robot to the last path point
double get_goal_distance(const double* path_x, const double* path_y,
                         int n_points, double x_V, double y_V)
{
    double dx = path_x[n_points - 1] - x_V;
    double dy = path_y[n_points - 1] - y_V;
    return std::sqrt(dx*dx + dy*dy);
}

// remaining path length from the closest point to the last point
double get_remaining_path_length(const double* path_x, const double* path_y,
                                 int n_points, int closest_idx)
{
    double l_rem = 0.0;
    for (int i = closest_idx; i < n_points - 1; i++)
    {
        double dx = path_x[i + 1] - path_x[i];
        double dy = path_y[i + 1] - path_y[i];
        l_rem += std::sqrt(dx*dx + dy*dy);
    }
    return l_rem;
}

// straight-line distance from the robot to the lookahead point
double get_lookahead_distance_to_point(double x_L, double y_L,
                                       double x_V, double y_V)
{
    double dx = x_L - x_V;
    double dy = y_L - y_V;
    return std::sqrt(dx*dx + dy*dy);
}

// Feedforward heading rate: v * (dtheta_path / ds) at the look-ahead
// point. This is how fast the reference heading is turning along the
// path, converted to an angular velocity via the current speed. Feeding
// it forward lets the robot rotate WITH the path instead of lagging and
// relying on the proportional term to catch up.
double get_heading_feedforward(const double* path_theta, int idx,
                               int n_points, const double* path_x,
                               const double* path_y, double v)
{
    if (idx < 0) idx = 0;
    if (idx >= n_points - 1)
        return 0.0;                 // no segment ahead -> no path rotation

    double dth = path_theta[idx + 1] - path_theta[idx];
    while (dth >  M_PI) dth -= 2.0 * M_PI;
    while (dth < -M_PI) dth += 2.0 * M_PI;

    double dx = path_x[idx + 1] - path_x[idx];
    double dy = path_y[idx + 1] - path_y[idx];
    double ds = std::sqrt(dx * dx + dy * dy);
    if (ds < 1e-6)
        return 0.0;

    // omega_ff = v * (dtheta/ds).  Units (m/s)*(rad/m) = rad/s.
    return v * (dth / ds);
}

double get_angular_velocity(Path_Tracker *pt, Init *init,
                            double theta_L, double theta_V, double d,
                            double v_ref, double omega_ff)
{
    // FEEDFORWARD + PROPORTIONAL heading law:
    //
    //     omega = omega_ff + KP_HEADING * heading_error
    //
    // omega_ff (feedforward) is the path's own heading rate at the
    // look-ahead point: how fast the reference heading is changing as the
    // robot moves along the path. It makes the robot rotate WITH the path
    // without waiting for an error to build up. It is computed by the
    // caller as v * (dtheta_path / ds) and passed in.
    //
    // KP_HEADING * heading_error is a plain fixed-gain proportional
    // correction of the current heading error. It does NOT scale with
    // speed or distance -- it is a constant gain, which is simple to tune
    // and easy to reason about. (This replaces the old time-ratio k form.)
    (void)v_ref;   // no longer used: correction is a fixed gain, not speed-scaled
    (void)d;       // no longer used: feedforward+P law does not divide by distance
    double kp        = pt->KP_HEADING;
    double omega_max = pt->MAXIMUM_ANGULAR_VELOCITY;
    double alpha_max = pt->MAXIMUM_ANGULAR_ACCELERATION;
    double dt        = 1.0 / pt->CONTROLLER_FREQUENCY;

    // heading error, wrapped to (-pi, pi]
    double angle_diff = theta_L - theta_V;
    while (angle_diff >  M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // TARGET omega for this cycle (before the accel limit). In the
    // deadband we drop the proportional term but KEEP the feedforward, so
    // the robot still tracks the path's rotation even when the error is
    // tiny. We do not force omega to 0; the acceleration clamp below
    // brings it toward the target smoothly.
    double omega_target = omega_ff;
    if (std::fabs(angle_diff) >= pt->YAW_GOAL_TOLERANCE)
        omega_target += kp * angle_diff;

    // cap the TARGET by the max angular velocity
    omega_target = std::clamp(omega_target, -omega_max, omega_max);

    // acceleration limit: omega moves at most alpha_max*dt per cycle
    // toward the target -- smooth spin-up AND spin-down.
    double omega = std::clamp(omega_target,
                              init->OMEGA_PREV - alpha_max * dt,
                              init->OMEGA_PREV + alpha_max * dt);

    init->OMEGA_PREV = omega;
    return omega;
}

double get_linear_velocity(Path_Tracker *pt, Init *init,
                           double d_g, double l_rem, double v_cap)
{
    double v_des     = pt->V;
    double a_max     = pt->MAXIMUM_LINEAR_ACCELERATION;
    double a_brake   = pt->CORNER_DECEL;   // harder rate used ONLY to slow down
    double dt        = 1.0 / pt->CONTROLLER_FREQUENCY;
    double D_app     = pt->APPROACH_VELOCITY_SCALING_DISTANCE;
    double v_app_min = pt->MINIMUM_APPROACH_LINEAR_VELOCITY;
    double v_min     = pt->MINIMUM_LINEAR_VELOCITY;

    // corner slowdown caps the DESIRED speed.
    v_des = std::min(v_des, v_cap);

    // Two different rates:
    //   - SPEEDING UP  -> gentle a_max (smooth, not jumpy everywhere)
    //   - SLOWING DOWN -> hard a_brake = CORNER_DECEL (brake late, close to
    //     the corner, without making the whole robot aggressive)
    // We decide by whether the desired speed is below the current speed.
    double v;
    if (v_des < init->V_PREV)
        v = std::max(v_des, init->V_PREV - a_brake * dt);   // decelerate hard
    else
        v = std::min(v_des, init->V_PREV + a_max * dt);     // accelerate gently

    // approach-to-goal scaling near the end (uses the hard brake rate too,
    // so the final stop is crisp)
    if (l_rem < D_app) {
        double v_app = std::max(v_des * (d_g / D_app), v_app_min);
        if (v_app < init->V_PREV)
            v = std::min(v, std::max(v_app, init->V_PREV - a_brake * dt));
        else
            v = std::min(v, v_app);
    }

    v = std::max(v, v_min);

    init->V_PREV = v;

    return v;
}

// ---- curvature-aware speed cap -------------------------------------------
// Walk the path forward from the closest point, accumulating arc length and
// heading change. If, within the slowdown window (which grows with speed),
// the path bends by more than CORNER_ANGLE_THRESHOLD, a corner is ahead:
// return CORNER_VELOCITY. Otherwise no corner near -> return pt->V.
double get_corner_speed_limit(Path_Tracker *pt, Init *init,
                              const double* path_x, const double* path_y,
                              int n_points, int closest_idx)
{
    if (!pt->CORNER_DETECTION)
        return pt->V;                 // feature off -> no cap (old behaviour)

    if (closest_idx < 0) closest_idx = 0;
    if (closest_idx >= n_points - 1)
        return pt->V;                 // at/after the last segment

    // Window must be at least the distance needed to decelerate from the
    // current speed down to CORNER_VELOCITY at the configured accel limit,
    // or the corner is seen too late to slow down in time. We take the max
    // of that braking distance and the user's MIN_CORNER_SLOWDOWN_DISTANCE, plus
    // a speed-scaled margin.
    const double v_now = std::max(0.0, init->V_PREV);
    double brake_dist = 0.0;
    if (v_now > pt->CORNER_VELOCITY && pt->CORNER_DECEL > 1e-6) {
        brake_dist = (v_now * v_now - pt->CORNER_VELOCITY * pt->CORNER_VELOCITY)
                     / (2.0 * pt->CORNER_DECEL);
    }
    const double window = std::max(pt->MIN_CORNER_SLOWDOWN_DISTANCE, brake_dist)
                        + v_now * pt->LOOKAHEAD_TIME;

    double arc      = 0.0;
    double turn     = 0.0;
    double prev_seg = std::atan2(path_y[closest_idx + 1] - path_y[closest_idx],
                                 path_x[closest_idx + 1] - path_x[closest_idx]);

    for (int i = closest_idx + 1; i < n_points - 1; i++) {
        const double dx = path_x[i + 1] - path_x[i];
        const double dy = path_y[i + 1] - path_y[i];
        const double seg_len = std::sqrt(dx * dx + dy * dy);
        if (seg_len < 1e-9) continue;

        const double seg_ang = std::atan2(dy, dx);
        double dth = seg_ang - prev_seg;
        while (dth >  M_PI) dth -= 2.0 * M_PI;
        while (dth <= -M_PI) dth += 2.0 * M_PI;

        turn += std::fabs(dth);
        arc  += seg_len;
        prev_seg = seg_ang;

        // corner found within the window -> slow down
        if (turn >= pt->CORNER_ANGLE_THRESHOLD)
            return pt->CORNER_VELOCITY;

        if (arc >= window)
            break;                    // scanned far enough, no corner near
    }

    return pt->V;                     // straight ahead within the window
}

// world-frame velocity aimed at the lookahead point
void get_world_velocity(Init *init, double v, double d,
                        double x_L, double y_L, double x_V, double y_V)
{
    init->VX_WORLD = 0.0;
    init->VY_WORLD = 0.0;
    if (d > 1e-9)
    {
        init->VX_WORLD = v * (x_L - x_V) / d;
        init->VY_WORLD = v * (y_L - y_V) / d;
    }
}

// rotate a world-frame velocity into the robot frame
void world_to_robot_frame_velocity(double vx_world, double vy_world,
                                   double theta_V,
                                   double* vx_body, double* vy_body)
{
    double c = std::cos(-theta_V);
    double s = std::sin(-theta_V);
    *vx_body = c * vx_world - s * vy_world;
    *vy_body = s * vx_world + c * vy_world;
}