#include "pathway.h"

using namespace std;
using namespace tk;

Pathway::Pathway(vector<double> x_waypoints, vector<double> y_waypoints, vector<double> s_waypoints) {
  coarse_x_ = x_waypoints;
  coarse_y_ = y_waypoints;
  coarse_s_ = s_waypoints;
}

void Pathway::InitSpline() {
  spline_x_.set_points(coarse_s_, coarse_x_);
  spline_y_.set_points(coarse_s_, coarse_y_);
}

void Pathway::RefineSpline() {
  const int samples = int(coarse_s_[coarse_s_.size()-1]);
  fine_x_.reserve(samples);
  fine_y_.reserve(samples);
  fine_s_.reserve(samples);
  for (int i = 0; i < samples; i++) {
    fine_x_.push_back(spline_x_(i));
    fine_y_.push_back(spline_y_(i));
    fine_s_.push_back(i);
  }
}
