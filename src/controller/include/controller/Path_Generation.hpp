#ifndef PATH_GENERATION_HPP
#define PATH_GENERATION_HPP

#include <cmath>
#include <vector>

// A single pose on the path: position in m, heading in rad
typedef struct Pose2D
{
    double x;
    double y;
    double theta;
} Pose2D;

// A dense path: one heading per point, ready for get_lookahead_point()
typedef struct Path
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;

    int size() const { return int(x.size()); }
} Path;

// Parameters for path generation, adjust here in one place
typedef struct Path_Params
{
    double STEP;   // spacing between dense poses in m (planner resolution)
} Path_Params;

void   path_params_init(Path_Params *pp);

// wrap angle to (-pi, pi]
double normalize_angle(double a);

// dense path from pose a to pose b: positions every STEP metres,
// heading interpolated per point along the segment
void   build_dense_path(const Pose2D *a, const Pose2D *b,
                        const Path_Params *pp, Path *path);


// dense circular arc from the circle equation:
// centre (cx, cy), radius R, from angle a0 to a1 in rad
// (CCW if a1 > a0). tangent_heading = 1: heading follows the tangent
// plus heading_offset; tangent_heading = 0: heading fixed at
// heading_offset (pure swerve, body does not rotate).
void   build_circle_path(double cx, double cy, double R,
                         double a0, double a1,
                         double heading_offset, int tangent_heading,
                         const Path_Params *pp, Path *path);

#endif // PATH_GENERATION_HPP