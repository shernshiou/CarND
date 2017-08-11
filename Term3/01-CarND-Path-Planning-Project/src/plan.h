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
#include "spline.h"

#define INC_MAX 0.4425 // max speed ~ 49.75MPH

using namespace std;
using json = nlohmann::json;

struct Path {
    std::vector<double> s_start;
    std::vector<double> s_end;
    std::vector<double> d_start;
    std::vector<double> d_end;
};

struct Trajectory {
    vector<double> x_vals;
    vector<double> y_vals;
    vector<double> s_vals;
    vector<double> d_vals;
};

enum State {
  READY,
  KEEP_LANE,
  PREPARE_CHANGE
};

class Plan {
 private:
  Pathway road_map_;
  MainVehicle car_;
  State current_state_;
  double ref_velocity_;
  std::array<double, 3> lane_cost_;

 public:
  Plan() = default;
  Plan(Pathway road_map);
  ~Plan() = default;

  Trajectory Follow();
  Trajectory Follow(double car_s, vector<double> &next_x_vals, vector<double> &next_y_vals);
  Trajectory GenTrajectory(vector<double> previous_path_x, vector<double> previous_path_y);
  Trajectory MinimumJerk(Path proposed_path);
  vector<double> CalcMinimumJerk(vector<double> start, vector<double> end);
  void CalcCost();
  void NextTrajectory(json j, vector<double> &next_x_vals, vector<double> &next_y_vals);

  double ComputeCarVelocity();
};

#endif /* PLAN_H_ */
