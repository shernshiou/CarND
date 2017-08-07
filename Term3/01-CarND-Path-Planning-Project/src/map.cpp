#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "map.h"

Map::Map() {}

Map::Map(vector<double> x_waypoints, vector<double> y_waypoints, vector<double> s_waypoints) {
  this->coarse_x = x_waypoints;
  this->coarse_y = y_waypoints;
  this->coarse_s = s_waypoints;
}

Map::~Map() {}

void Map::InitSpline() {
  this->spline_x.set_points(this->coarse_s, this->coarse_x);
  this->spline_y.set_points(this->coarse_s, this->coarse_y);
}

void Map::RefineSpline() {
  const int samples = int(this->coarse_s[this->coarse_s.size()-1]);
  this->fine_x.reserve(samples);
  this->fine_y.reserve(samples);
  this->fine_s.reserve(samples);
  for (int i = 0; i < samples; i++) {
    this->fine_x.push_back(this->spline_x(i));
    this->fine_y.push_back(this->spline_y(i));
    this->fine_s.push_back(i);
  }
}
