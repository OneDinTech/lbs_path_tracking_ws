#include "Path_Generation.hpp"

void path_params_init(Path_Params *pp)
{
    pp->STEP = 0.05;   // 5 cm, same density as a real SMAC path
}

// បង្វិលមុំឱ្យស្ថិតក្នុងជួរ (-π, π]
double normalize_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// បង្កើតគន្លងក្រាស់ពីចំណុច a ទៅចំណុច b
// ទីតាំងគ្រប់ STEP ម៉ែត្រ និងមុំត្រូវបាន interpolate តាមចម្ងាយ
void build_dense_path(const Pose2D *a, const Pose2D *b,
                      const Path_Params *pp, Path *path)
{
    path->x.clear();
    path->y.clear();
    path->theta.clear();

    // ចំណុចដំបូងគឺចំណុចចាប់ផ្ដើម
    path->x.push_back(a->x);
    path->y.push_back(a->y);
    path->theta.push_back(a->theta);

    // ភាពខុសគ្នានៃមុំតាមផ្លូវខ្លីបំផុត
    double dth     = normalize_angle(b->theta - a->theta);
    double seg_len = std::hypot(b->x - a->x, b->y - a->y);
    int    n       = std::max(int(seg_len / pp->STEP), 1);

    for (int k = 1; k <= n; k++)
    {
        double t = double(k) / n;
        path->x.push_back(a->x + t * (b->x - a->x));
        path->y.push_back(a->y + t * (b->y - a->y));
        path->theta.push_back(normalize_angle(a->theta + t * dth));
    }
}

// បង្កើតគន្លងរង្វង់ដោយប្រើសមីការរង្វង់ x = cx + R cos(a), y = cy + R sin(a)
void build_circle_path(double cx, double cy, double R,
                       double a0, double a1,
                       double heading_offset, int tangent_heading,
                       const Path_Params *pp, Path *path)
{
    path->x.clear();
    path->y.clear();
    path->theta.clear();

    // ប្រវែងធ្នូ = R * |a1 - a0| ចែកនឹង STEP ដើម្បីរកចំនួនចំណុច
    double arc_len = R * std::fabs(a1 - a0);
    int    n       = std::max(int(arc_len / pp->STEP), 1);

    for (int k = 0; k <= n; k++)
    {
        double a = a0 + (a1 - a0) * double(k) / n;
        path->x.push_back(cx + R * std::cos(a));
        path->y.push_back(cy + R * std::sin(a));

        if (tangent_heading)
        {
            // មុំតង់សង់នៃរង្វង់: a + π/2 សម្រាប់ CCW, a - π/2 សម្រាប់ CW
            double tangent = a + (a1 > a0 ? M_PI / 2.0 : -M_PI / 2.0);
            path->theta.push_back(normalize_angle(tangent + heading_offset));
        }
        else
        {
            // មុំថេរ: រាងកាយរបស់រ៉ូបូតមិនបង្វិលទេ (pure swerve)
            path->theta.push_back(normalize_angle(heading_offset));
        }
    }
}