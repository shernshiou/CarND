#include "plan.h"

Plan::Plan(Pathway road_map) {
  road_map_ = road_map;
  road_map_.InitSpline();
  road_map_.RefineSpline();
  current_state_ = State::READY;
}

void Plan::NextTrajectory(json j, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    // Previous path data given to the Planner
    vector<double> previous_path_x = j[1]["previous_path_x"];
    vector<double> previous_path_y = j[1]["previous_path_y"];
    auto sensor_fusion = j[1]["sensor_fusion"];

    // cout << previous_path_x.size() << endl;
    if (previous_path_x.size() == 0 && current_state_ == State::READY) {
        car_ = MainVehicle(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);
        car_.ComputeGap(sensor_fusion);
        car_.PrintGap();
        Trajectory traject = KeepLane();
        next_x_vals = traject.x_vals;
        next_y_vals = traject.y_vals;
        current_state_ = State::KEEP_LANE;
    } else if (previous_path_x.size() > 30) {
        car_.Update(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);
        car_.ComputeGap(sensor_fusion);
        car_.PrintGap();
        next_x_vals = previous_path_x;
        next_y_vals = previous_path_y;
    } else {
        car_.Update(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);
        car_.ComputeGap(sensor_fusion);
        car_.PrintGap();
        Trajectory traject = KeepLane();
        next_x_vals = traject.x_vals;
        next_y_vals = traject.y_vals;
        current_state_ = State::KEEP_LANE;
    }
}

Trajectory Plan::KeepLane() {
    double car_last_s = car_.s_;
    double dist_inc = 0.4;
    Trajectory traject;
    for(int i = 0; i < 50; i++)
    {
      traject.x_vals.push_back(road_map_.spline_x_(car_last_s+(dist_inc*i)));
      traject.y_vals.push_back(road_map_.spline_y_(car_last_s+(dist_inc*i))-6);
    }

    return traject;
}

void Plan::Follow(double car_s, vector<double> &next_x_vals, vector<double> &next_y_vals) {
  double car_last_s = car_s;
  double dist_inc = 0.4;

  for(int i = 0; i < 50; i++)
  {
    next_x_vals.push_back(road_map_.spline_x_(car_last_s+(dist_inc*i)));
    next_y_vals.push_back(road_map_.spline_y_(car_last_s+(dist_inc*i))-6);
  }
}
