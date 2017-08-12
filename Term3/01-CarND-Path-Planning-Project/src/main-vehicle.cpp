#include "main-vehicle.h"

using namespace std;

void MainVehicle::Update(double x, double y, double s, double d, double yaw, double speed) {
    Vehicle::Update(x, y, s, d, yaw, speed);
    gap_front_ = {99999, 99999, 99999, 99999};
    gap_rear_ = {99999, 99999, 99999, 99999};
    // lane_ = Util::DToLane(s_);
}

void MainVehicle::ComputeGap(vector<vector<double>> sensor_fusion, Pathway road_map, int size) {
    sensor_fusion_ = sensor_fusion;
    gap_front_ = {99999, 99999, 99999, 99999};
    gap_rear_ = {99999, 99999, 99999, 99999};
    close_cars_.clear();

    if (sensor_fusion_.size() > 0) {
        for (vector<double> data : sensor_fusion_) {
            int id = (int)data[0];
            Vehicle other_vehicle = Vehicle(id, data[1], data[2], data[3], data[4], data[5], data[6]); // [ id, x, y, vx, vy, s, d]
            double check_car_s = other_vehicle.s_;
            check_car_s += ((double)size*.02*other_vehicle.speed_);
            double distance = check_car_s-s_;

            int lane = 0;
            if (other_vehicle.d_ >= 0 && other_vehicle.d_ < (2+4-2)) {
                lane = 0;
            }
            if (other_vehicle.d_ >= (2+4-2) && other_vehicle.d_ < (2+4+2)) {
                lane = 1;
            }
            if (other_vehicle.d_ > (2+4+2)) {
                lane = 2;
            }

            if ((other_vehicle.s_ - s_) > 0.0 && distance < gap_front_[lane]) {
                gap_front_[lane] = distance;
                closest_front_[lane] = other_vehicle;
            }
            if ((other_vehicle.s_ - s_) < 0.0 && distance < gap_rear_[lane]) {
                gap_rear_[lane] = -distance;
                closest_rear_[lane] = other_vehicle;
            }
        }
    }
}

void MainVehicle::PrintGap() {
    cout << "Front: "
         << gap_front_[0] << " [" << closest_front_[0].speed_ << "] "
         << gap_front_[1] << " [" << closest_front_[1].speed_ << "] "
         << gap_front_[2] << " [" << closest_front_[2].speed_ << endl;
    cout << "Rear: "
         << gap_rear_[0] << " [" << closest_rear_[0].speed_ << "] "
         << gap_rear_[1] << " [" << closest_rear_[1].speed_ << "] "
         << gap_rear_[2] << " [" << closest_rear_[2].speed_ << endl;
}
