#ifndef PATHWAY_H
#define PATHWAY_H

#include "spline.h"

class Pathway {
 public:
    std::vector<double> coarse_x_, coarse_y_, coarse_s_;
    std::vector<double> fine_x_, fine_y_, fine_s_;
    Pathway() = default;
    Pathway(std::vector<double> x_waypoints, std::vector<double> y_waypoints, std::vector<double> s_waypoints);
    ~Pathway() = default;
    tk::spline spline_x_, spline_y_, spline_s_;

    void InitSpline();
    void RefineSpline();
};

#endif /* PATHWAY_H_ */
