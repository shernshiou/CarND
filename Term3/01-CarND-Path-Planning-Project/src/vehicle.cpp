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
}
