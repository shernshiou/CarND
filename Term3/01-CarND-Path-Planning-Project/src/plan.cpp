#include "plan.h"

Plan::Plan(Pathway road_map) {
  ref_velocity_ = 0.0;
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

    Trajectory traject;
    if (previous_path_x.size() == 0 && current_state_ == State::READY) {
        car_ = MainVehicle(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);
        car_.Update(1);
        car_.ComputeGap(sensor_fusion, road_map_, previous_path_x.size());
        // car_.PrintGap();
        traject = GenTrajectory(previous_path_x, previous_path_y);
        current_state_ = State::KEEP_LANE;
    } else {
        car_.Update(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);
        car_.ComputeGap(sensor_fusion, road_map_, previous_path_x.size());
        car_.PrintGap();
        traject = GenTrajectory(previous_path_x, previous_path_y);
    }

    next_x_vals = traject.x_vals;
    next_y_vals = traject.y_vals;
}

Trajectory Plan::GenTrajectory(vector<double> previous_path_x, vector<double> previous_path_y) {
    Trajectory traject;
    vector<double> ptsx, ptsy;
    int prev_size = previous_path_x.size();
    double ref_x = car_.x_;
    double ref_y = car_.y_;
    double ref_yaw = Util::deg2rad(car_.yaw_);

    if (prev_size < 2) {
        double prev_car_x = car_.x_ - cos(car_.yaw_);
        double prev_car_y = car_.y_ - sin(car_.yaw_);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_.x_);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_.y_);
    } else {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];
        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    if (ref_velocity_ > 1 && car_.gap_front_[car_.lane_] < 40) {
        if (ref_velocity_ > 20)
            ref_velocity_ -= .324;
        cout << "Prepare to change" << ":" << ref_velocity_ << endl;
        current_state_ = State::PREPARE_CHANGE;

        if (ref_velocity_ < 32 && current_state_ == State::PREPARE_CHANGE) {
            int left_lane = car_.lane_ -1;
            int right_lane = car_.lane_ +1;

            double cost_left = 0;
            double cost_right = 0;

            // Cost of Left
            if (left_lane >= 0 && car_.gap_front_[left_lane] > 30 && car_.gap_rear_[left_lane] > 10) {
                cost_left += (int)car_.gap_front_[left_lane];
                cost_left += (int)car_.gap_rear_[left_lane];
            }

            // Cost of Right
            if (right_lane <=2 && car_.gap_front_[right_lane] > 30 && car_.gap_rear_[right_lane] > 10) {
                cost_right += (int)car_.gap_front_[right_lane];
                cost_right += (int)car_.gap_rear_[right_lane];
            }

            cost_left -= 0.002; // Always prefer left if similar

            if (cost_left > 0 && cost_left > cost_right) {
                car_.lane_ = left_lane;
                current_state_ = State::KEEP_LANE;
            } else if (cost_right > 0 && cost_right > cost_left) {
                car_.lane_ = right_lane;
                current_state_ = State::KEEP_LANE;
            }
        }
    } else if (ref_velocity_ < 49.5){
        ref_velocity_ += .224;
        cout << "Keep Lane" << endl;
        current_state_ = State::KEEP_LANE;
    }

    vector<double> next_wp0 = Util::getXY(car_.s_+30, Util::LaneToD(car_.lane_), road_map_.coarse_s_, road_map_.coarse_x_, road_map_.coarse_y_);
    vector<double> next_wp1 = Util::getXY(car_.s_+60, Util::LaneToD(car_.lane_), road_map_.coarse_s_, road_map_.coarse_x_, road_map_.coarse_y_);
    vector<double> next_wp2 = Util::getXY(car_.s_+90, Util::LaneToD(car_.lane_), road_map_.coarse_s_, road_map_.coarse_x_, road_map_.coarse_y_);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;
        ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals, next_y_vals;
    for(int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_add_on = 0;

    for (int i = 1; i <= 50-prev_size; i++) {
        double N = (target_dist/(.02*ref_velocity_/2.24));
        double x_point = x_add_on+(target_x)/N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;
        x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    traject.x_vals = next_x_vals;
    traject.y_vals = next_y_vals;
    return traject;
}

// Trajectory Plan::KeepLane() {
//     // double car_last_s = car_.s_;
//     // double dist_inc = 0.4;
//     // Trajectory traject;
//     // for(int i = 0; i < 50; i++)
//     // {
//     //   traject.x_vals.push_back(road_map_.spline_x_(car_last_s+(dist_inc*i)));
//     //   traject.y_vals.push_back(road_map_.spline_y_(car_last_s+(dist_inc*i))-6);
//     // }
//     //
//     // return traject;
//
//     double optimum_speed = car_.ComputeOptimumSpeed();
//     double s_end = car_.speed_ + 2.5 * 0.5 * (car_.speed_ + optimum_speed);
//
//     return MinimumJerk(Path{
//         { car_.s_, car_.speed_, 0 },
//         { car_.s_ + 50, 30, 0 },
//         { Util::LaneToD(2), 0, 0 },
//         { Util::LaneToD(2), 0, 0 }
//     });
//
//     // return MinimumJerk(Path{
//     //     { car_.s_, car_.speed_, 0 },
//     //     { s_end, 40, 0 },
//     //     { Util::LaneToD(car_.lane_), 0, 0 },
//     //     { Util::LaneToD(car_.lane_), 0, 0 }
//     // });
// }

Trajectory Plan::MinimumJerk(Path proposed_path) {
    double t = 2.5;
    // Generate minimum jerk path in Frenet coordinates
    // vector<double> next_s_vals = Util::JMT(proposed_path.s_start, proposed_path.s_end, t);
    // vector<double> next_d_vals = Util::JMT(proposed_path.d_start, proposed_path.d_end, t);

    vector<double> next_s_vals = CalcMinimumJerk(proposed_path.s_start, proposed_path.s_end);
    vector<double> next_d_vals = CalcMinimumJerk(proposed_path.d_start, proposed_path.d_end);

    Trajectory traject;

    for (int i=0; i<next_s_vals.size(); i++) {
        vector<double> xy = Util::getXY(fmod(next_s_vals[i], 6945.554),
                                  next_d_vals[i],
                                  road_map_.fine_s_,
                                  road_map_.fine_x_,
                                  road_map_.fine_y_);
        traject.x_vals.push_back(xy[0]);
        traject.y_vals.push_back(xy[1]);
    }

    return traject;
}

vector<double> Plan::CalcMinimumJerk(vector<double> start, vector<double> end) {
    #define PATH_HORIZON 2.5
    #define PATH_WP_PERIOD 0.02

    MatrixXd A = MatrixXd(3,3);
    VectorXd b = VectorXd(3);
    VectorXd x = VectorXd(3);

    double t  = PATH_HORIZON;
    double t2 = t * t;
    double t3 = t * t2;
    double t4 = t * t3;
    double t5 = t * t4;

    A <<   t3,    t4,    t5,
         3*t2,  4*t3,  5*t4,
         6*t,  12*t2, 20*t3;

    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
         end[1] - (start[1] + start[2] * t),
         end[2] - start[2];

    x = A.inverse() * b;

    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2.0;
    double a3 = x[0];
    double a4 = x[1];
    double a5 = x[2];

    vector<double> result;
    for (double t=PATH_WP_PERIOD; t<PATH_HORIZON+0.001; t+=PATH_WP_PERIOD)
    {
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t * t3;
        double t5 = t * t4;
        double r = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        result.push_back(r);
    }
    return result;
}

Trajectory Plan::Follow() {
    Trajectory traject;
    double dist_inc = 0.5;

    for(int i = 0; i < 50; i++)
    {
      double next_s = car_.s_+(i+1)*dist_inc;
      double next_d = 6;
      vector<double> xy = Util::getXY(next_s, next_d, road_map_.coarse_s_, road_map_.coarse_x_, road_map_.coarse_y_);
    //   next_x_vals.push_back(xy[0]);
    //   next_y_vals.push_back(xy[1]);
        traject.x_vals.push_back(xy[0]);
        traject.y_vals.push_back(xy[1]);
    }

    return traject;
}

// Trajectory Plan::Follow(double car_s, vector<double> &next_x_vals, vector<double> &next_y_vals) {
//     Trajectory traject;
//     double dist_inc = 0.5;
//
//     for(int i = 0; i < 50; i++)
//     {
//       double next_s = car_s+(i+1)*dist_inc;
//       double next_d = 6;
//       vector<double> xy = Util::getXY(next_s, next_d, road_map_.coarse_s_, road_map_.coarse_x_, road_map_.coarse_y_);
//     //   next_x_vals.push_back(xy[0]);
//     //   next_y_vals.push_back(xy[1]);
//         traject.x_vals.push_back(xy[0]);
//         traject.y_vals.push_back(xy[1]);
//     }
//
//     return traject;
// }
