#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <vector>
#include <array>
#include <map>
#include "util.h"
#include "pathway.h"

class Vehicle {
 public:
    Vehicle() = default;
    Vehicle(double x, double y, double s, double d, double yaw, double speed):
        id_(0), x_(x), y_(y), s_(s), d_(d), yaw_(yaw), speed_(speed) {};
    Vehicle(int id, double x, double y, double vx, double vy, double s, double d):
        id_(id), x_(x), y_(y), s_(s), d_(d), vx_(vx), vy_(vy) {
            speed_ = sqrt(pow(vx, 2)+pow(vy, 2));
        };
    ~Vehicle() = default;

    int id_, lane_;
    double x_, y_, s_, d_, yaw_, speed_;
    double vx_, vy_;
    // double speed_;

    void Update(double x, double y, double s, double d, double yaw, double speed);
    void Update(int lane, double x, double y, double s, double d, double vx, double vy);
    void Update(int lane);
    void ComputeCarSpeed(Pathway road_map);
};

#endif /* VEHICLE_H_ */
