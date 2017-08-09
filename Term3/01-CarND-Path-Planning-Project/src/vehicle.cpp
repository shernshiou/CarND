#include "vehicle.h"

using namespace std;

void Vehicle::Update(double x, double y, double s, double d, double yaw, double speed) {
    x_ = x;
    y_ = y;
    s_ = s;
    d_ = d;
    yaw_ = yaw;
    speed_ = speed;
}

void Vehicle::Update(int lane, double x, double y, double s, double d, double vx, double vy) {
    lane_ = lane;
    x_ = x;
    y_ = y;
    s_ = s;
    d_ = d;
    vx_ = vx;
    vy_ = vy;
    speed_ = sqrt(pow(vx, 2)+pow(vy, 2));
}

void Vehicle::Update(int lane) {
    lane_ = lane;
}

void Vehicle::ComputeCarSpeed(Pathway road_map) {
    double car_heading  = Util::rad2deg(atan(vy_/vx_));
    int waypoint0 = Util::ClosestWaypoint(x_, y_, road_map.fine_x_, road_map.fine_y_);
	int waypoint1 = Util::NextWaypoint(x_, y_, car_heading, road_map.fine_x_, road_map.fine_y_);
    double lane_heading = Util::rad2deg(
        atan((road_map.fine_y_[waypoint1] - road_map.fine_y_[waypoint0]) /
            (road_map.fine_x_[waypoint1] - road_map.fine_x_[waypoint0])));
    double delta_theta = car_heading - lane_heading;
    double mag_v = sqrt(pow(vx_,2) + pow(vy_,2));
	double v_s = mag_v * cos(delta_theta);
	double v_d = mag_v * sin(delta_theta);
    speed_ = mag_v;
}
