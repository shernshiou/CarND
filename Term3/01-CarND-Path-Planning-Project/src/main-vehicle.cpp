#include "main-vehicle.h"

using namespace std;

void MainVehicle::Update(double x, double y, double s, double d, double yaw, double speed) {
    Vehicle::Update(x, y, s, d, yaw, speed);
    gap_front_ = {99999, 99999, 99999, 99999};
    gap_rear_ = {99999, 99999, 99999, 99999};
    lane_ = Util::DToLane(s_);
}

void MainVehicle::ComputeGap(vector<vector<double>> sensor_fusion, Pathway road_map) {
    sensor_fusion_ = sensor_fusion;
    close_cars_.clear();

    if (sensor_fusion_.size() > 0) {
        for (vector<double> data : sensor_fusion_) {
            int id = (int)data[0];
            Vehicle other_vehicle = Vehicle(id, data[1], data[2], data[3], data[4], data[5], data[6]); // [ id, x, y, vx, vy, s, d]
            // other_vehicle.ComputeCarSpeed(road_map);
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

double MainVehicle::ComputeOptimumSpeed() {
    return ComputeOptimumSpeed(lane_);
}

double MainVehicle::ComputeOptimumSpeed(int lane) {
    double front_distance = gap_front_[lane];
    double front_car_speed = closest_front_[lane].speed_;
    double speed_adj  = KL_ACC_COEFF * front_distance;

	speed_adj = speed_adj > MAX_SPEED_VAR ? MAX_SPEED_VAR : speed_adj;
	speed_adj = speed_adj < MIN_SPEED_VAR ? MIN_SPEED_VAR : speed_adj;

	double target_speed = speed_ + speed_adj;

    // Adjust computed target speed decreasing it in order to reach the same speed of the car in front at the distance of
	if(target_speed > speed_){
		if(front_distance > FORWARD_COLLISION_BUFFER*3)
			return target_speed;
		else if(front_distance >= FORWARD_COLLISION_BUFFER)
		{
			return speed_ + (target_speed-front_car_speed)* ((front_distance-FORWARD_COLLISION_BUFFER)/(3*FORWARD_COLLISION_BUFFER));
		}
		else if(front_distance < FORWARD_COLLISION_BUFFER){
			return MIN_SPEED + (speed_ - MIN_SPEED)* ((front_distance)/(FORWARD_COLLISION_BUFFER));
		}
	}
	return target_speed;
}

void MainVehicle::PrintGap() {
    cout << "Front: "
         << gap_front_[0] << " [" << closest_front_[0].speed_ << "] "
         << gap_front_[1] << " [" << closest_front_[1].speed_ << "] "
         << gap_front_[2] << " [" << closest_front_[2].speed_ << "] "
         << gap_front_[3] << " [" << closest_front_[3].speed_ << "] " << endl;
    cout << "Rear: "
         << gap_rear_[0] << " [" << closest_rear_[0].speed_ << "] "
         << gap_rear_[1] << " [" << closest_rear_[1].speed_ << "] "
         << gap_rear_[2] << " [" << closest_rear_[2].speed_ << "] "
         << gap_rear_[3] << " [" << closest_rear_[3].speed_ << "] " << endl;
}
