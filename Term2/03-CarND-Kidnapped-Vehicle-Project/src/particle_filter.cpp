/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

const int NUM_PARTICLES = 1000;

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = NUM_PARTICLES;

	random_device rd;
	mt19937 gen(rd());

	// Normal distribution for x, y, and theta.
	normal_distribution<> d_x(x, std[0]);
	normal_distribution<> d_y(y, std[1]);
	normal_distribution<> d_theta(theta, std[2]);

	particles.resize(num_particles);
	weights.resize(num_particles);

	int idx = 0;
	for(auto particle : particles) {
		particle.id = idx;
		particle.x = d_x(gen);
		particle.y = d_y(gen);
		particle.theta = d_theta(gen);
		particle.weight = 1.0;
		weights[idx] = 1.0;

		idx += 1;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	random_device rd;
	mt19937 gen(rd());

	// Normal distribution for x, y, and theta.
	normal_distribution<> d_x(0.0, std_pos[0]);
	normal_distribution<> d_y(0.0, std_pos[1]);
	normal_distribution<> d_theta(0.0, std_pos[2]);

	for(auto particle : particles) {
		if(fabs(yaw_rate) < 0.0001) {
			particle.x += velocity*delta_t * cos(particle.theta);
			particle.y += velocity*delta_t * sin(particle.theta);
		} else {
			double phi = particle.theta + yaw_rate * delta_t;
			particle.x += velocity/yaw_rate * (sin(phi) - sin(particle.theta));
      particle.y += velocity/yaw_rate * (cos(particle.theta) - cos(phi));
			particle.theta = phi;
		}

		particle.x += d_x(gen);
		particle.y += d_y(gen);
		particle.theta += d_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predictions, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for(auto observation : observations) {
		double temp_delta_l = 0.0;
		bool temp_init = false;
		for(auto prediction : predictions) {
			double delta_x = observation.x - prediction.x;
			double delta_y = observation.y - prediction.y;
			double delta_l = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

			if((!temp_init) || (temp_delta_l > delta_l)) {
				temp_delta_l = delta_l;
				temp_init = true;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	int idx = 0;
	for(auto particle : particles) {
		double weight_new = 1.0;
		for(auto observation : observations) {
			// Transform local coordinate to global coordinate
			observation.x = observation.x * cos(particle.theta) - observation.y * sin(particle.theta) + particle.x;
			observation.y = observation.x * sin(particle.theta) + observation.y * cos(particle.theta) + particle.y;


		}

		idx++;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	random_device rd;
	mt19937 gen(rd());
	discrete_distribution<> d_particles(weights.begin(), weights.end());
	vector<Particle> particles_new;
  particles_new.resize(num_particles);

	for(auto particle : particles_new) {
		particle = particles[d_particles(gen)];
	}

	particles = particles_new;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
