#ifndef PLAN_H
#define PLAN_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <array>
#include <algorithm>
#include "pathway.h"
#include "util.h"
#include "vehicle.h"
#include "main-vehicle.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

struct Trajectory {
    vector<double>  x_vals;
    vector<double> y_vals;
};

enum State {
  READY,
  KEEP_LANE,
  CHANGE_RIGHT,
  CHANGE_LEFT
};

class Plan {
 private:
  Pathway road_map_;
  MainVehicle car_;
  State current_state_;

 public:
  Plan() = default;
  Plan(Pathway road_map);
  ~Plan() = default;

  void Follow(double car_s, vector<double> &next_x_vals, vector<double> &next_y_vals);
  Trajectory KeepLane();
  void NextTrajectory(json j, vector<double> &next_x_vals, vector<double> &next_y_vals);

  double ComputeCarVelocity();
};

#endif /* PLAN_H_ */
