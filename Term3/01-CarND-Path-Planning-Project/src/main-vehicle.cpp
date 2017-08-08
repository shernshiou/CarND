#include "main-vehicle.h"

using namespace std;

void MainVehicle::Update(double x, double y, double s, double d, double yaw, double speed) {
    Vehicle::Update(x, y, s, d, yaw, speed);
    gap_front_ = {99999, 99999, 99999, 99999};
    gap_rear_ = {99999, 99999, 99999, 99999};
}

void MainVehicle::ComputeGap(vector<vector<double>> sensor_fusion) {
    sensor_fusion_ = sensor_fusion;
    close_cars_.clear();

    if (sensor_fusion_.size() > 0) {
        for (vector<double> data : sensor_fusion_) {
            int id = (int)data[0];
            Vehicle other_vehicle = Vehicle(id, data[1], data[2], data[3], data[4], data[5], data[6]); // [ id, x, y, vx, vy, s, d]
            close_cars_[id] = other_vehicle;

            int lane = Util::DToLane(other_vehicle.d_);
            double distance = other_vehicle.s_ - s_;
            if (distance > 0.0 && distance < gap_front_[lane]) {
                gap_front_[lane] = distance;
                closest_front_[lane] = other_vehicle;
            }
            if (distance < 0.0 && -distance < gap_rear_[lane]) {
                gap_rear_[lane] = -distance;
                closest_rear_[lane] = other_vehicle;
            }
        }
    }
}

void MainVehicle::PrintGap() {
    cout << "Front: "
         << gap_front_[0] << " "
         << gap_front_[1] << " "
         << gap_front_[2] << " "
         << gap_front_[3] << endl;
    cout << "Rear: "
         << gap_rear_[0] << " "
         << gap_rear_[1] << " "
         << gap_rear_[2] << " "
         << gap_rear_[3] << endl;
}
