#ifndef MAIN_VEHICLE_H
#define MAIN_VEHICLE_H

#include "vehicle.h"

class MainVehicle : public Vehicle {
 public:
    MainVehicle()  = default;
    MainVehicle(double x, double y, double s, double d, double yaw, double speed) : Vehicle(x, y, s, d, yaw, speed) {};
    ~MainVehicle() = default;

    std::array<double, 4> gap_front_, gap_rear_;
    std::array<Vehicle, 4> closest_front_, closest_rear_;
    std::vector<std::vector<double>> sensor_fusion_;
    std::map<int, Vehicle> close_cars_;

    using Vehicle::Update;
    void Update(double x, double y, double s, double d, double yaw, double speed);
    void ComputeGap(std::vector<std::vector<double>> sensor_fusion, Pathway road_map, int size);
    void PrintGap();
};

#endif /* MAIN_VEHICLE_H_ */
