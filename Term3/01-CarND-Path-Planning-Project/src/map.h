#ifndef MAP_H
#define MAP_H

#include "spline.h"

using namespace std;
using namespace tk;

class Map {

public:

    Map();
    Map(vector<double> x_waypoints, vector<double> y_waypoints, vector<double> s_waypoints);
    virtual ~Map();

    tk::spline spline_x, spline_y, spline_s;
    vector<double> coarse_x, coarse_y, coarse_s;
    vector<double> fine_x, fine_y, fine_s;

    void InitSpline();
    void RefineSpline();

};
#endif /* MAP_H_ */
