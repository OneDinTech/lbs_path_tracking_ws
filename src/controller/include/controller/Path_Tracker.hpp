#ifndef PATH_TRACKER_HPP
#define PATH_TRACKER_HPP

#include <cmath>
#include <algorithm>

typedef struct Path_Tracker
{
    double MIN_LOOKAHEAD_DISTANCE;
    double MAX_LOOKAHEAD_DISTANCE;
    double LOOKAHEAD_TIME;
    double V;
    double KP_HEADING;   // fixed proportional heading gain (replaces time-ratio k)
    double MAXIMUM_LINEAR_ACCELERATION;
    double CONTROLLER_FREQUENCY;
    double APPROACH_VELOCITY_SCALING_DISTANCE;
    double MINIMUM_APPROACH_LINEAR_VELOCITY;
    double MINIMUM_LINEAR_VELOCITY;
    double MAXIMUM_ANGULAR_VELOCITY;
    double MAXIMUM_ANGULAR_ACCELERATION;
    double DISTANCE_GOAL_TOLERANCE;
    double YAW_GOAL_TOLERANCE;

    // ---- curvature-aware speed (corner slowdown) ----
    bool   CORNER_DETECTION;        // master switch; false = old behaviour
    double CORNER_VELOCITY;         // speed to hold around a corner (not 0)
    double CORNER_DECEL;            // deceleration used ONLY to brake into a corner
    double MIN_CORNER_SLOWDOWN_DISTANCE;// MINIMUM slow-down window (floor); braking distance usually dominates
    double CORNER_ANGLE_THRESHOLD;  // rad of cumulative turn that counts as a corner
} Path_Tracker;

typedef struct Init
{
    double OMEGA_PREV;
    double V_PREV;
    double VX_WORLD;
    double VY_WORLD;
} Init;

void   param_init(Init *init);
double get_lookahead_distance(Path_Tracker *pt, Init *init);
void   get_lookahead_point(const double* path_x, const double* path_y,
                           const double* path_theta,
                           int n_points, int start_idx,
                           double x_V, double y_V, double L,
                           double* x_L, double* y_L, double* theta_L,
                           int* closest_idx);
double get_goal_distance(const double* path_x, const double* path_y,
                         int n_points, double x_V, double y_V);
double get_remaining_path_length(const double* path_x, const double* path_y,
                                 int n_points, int closest_idx);
double get_lookahead_distance_to_point(double x_L, double y_L,
                                       double x_V, double y_V);
double get_angular_velocity(Path_Tracker *pt, Init *init,
                            double theta_L, double theta_V, double d,
                            double v_ref, double omega_ff);

// Feedforward heading rate from the path: how fast the reference heading
// changes at the look-ahead point, times the current speed. Returns
// v * (dtheta_path / ds). Pass the path heading array, the look-ahead
// index, the point count, and the current speed.
double get_heading_feedforward(const double* path_theta, int idx,
                               int n_points, const double* path_x,
                               const double* path_y, double v);
double get_linear_velocity(Path_Tracker *pt, Init *init,
                           double d_g, double l_rem, double v_cap);

// Scan the path ahead for a corner/curve. Returns a speed cap:
// CORNER_VELOCITY if a corner is within the (speed-scaled) slowdown window,
// otherwise pt->V. If CORNER_DETECTION is false, always returns pt->V.
double get_corner_speed_limit(Path_Tracker *pt, Init *init,
                              const double* path_x, const double* path_y,
                              int n_points, int closest_idx);
void   get_world_velocity(Init *init, double v, double d,
                          double x_L, double y_L, double x_V, double y_V);
void   world_to_robot_frame_velocity(double vx_world, double vy_world,
                                     double theta_V,
                                     double* vx_body, double* vy_body);

#endif // PATH_TRACKER_HPP