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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 50; 
	std::default_random_engine gen; 

	std::normal_distribution<double> noise_x(x, std[0]); 
	std::normal_distribution<double> noise_y(y, std[1]);
	std::normal_distribution<double> noise_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i){
		Particle particle; 
		particle.x = noise_x(gen); 
		particle.y = noise_y(gen); 
		particle.theta = noise_theta(gen); 

		particle.weight = 1; 
		weights.push_back(1.0); 
		particles.push_back(particle);
		 
	}
	
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

		std::default_random_engine gen;
		for (int i = 0; i < num_particles; ++i){
	// creating a gaussian distribution around the values to add  noise
			 
			std::normal_distribution<double> noise_x(0, std_pos[0]); 
			std::normal_distribution<double> noise_y(0, std_pos[1]);
			std::normal_distribution<double> noise_theta(0, std_pos[2]);

			particles[i].x += noise_x(gen); 
			particles[i].y += noise_y(gen); 
			particles[i].theta += noise_theta(gen); 

			// if the yaw rate is zero this is handled differently
			if(fabs(yaw_rate) > 0.00001){
				particles[i].x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)); 
				particles[i].y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)); 
				particles[i].theta = particles[i].theta + yaw_rate * delta_t; 
			} else{
				particles[i].x = particles[i].x + velocity * delta_t * cos(particles[i].theta); 
				particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta); 
				particles[i].theta = particles[i].theta; 
			}			
		}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// int smallest_index = num_particles + 1;  // initializing to something it cannot be  
	    // a big number 
	for(int i = 0; i < observations.size(); ++i){
		double smallest_distance = 100000000.0;
		for(int j = 0; j < predicted.size(); ++j){
			double d = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if(d < smallest_distance){
				smallest_distance = d;
				// smallest_index = j; 
				 
				observations[i].id = predicted[j].id;
				cout<<"observation ID: "<<observations[i].id<<"predicted landmark ID: "<<predicted[j].id<<endl;	
			}
			
		}	
		
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// looping through each particle to update its weight

	// cout<<"number of observations"<<observations[9].id<<endl; 
	// cout<<"landmarks "<<map_landmarks.landmark_list.size()<<endl; 
	// for(int lmark = 0; lmark <landmarks_within_sensor_range.size(); ++lmark ){
	// 	cout<<"LANDMARKS IN RANGE:"<<endl; 
	// 	cout<<landmarks_within_sensor_range[lmark].id<<"/"<<landmarks_within_sensor_range[lmark].x<<"/"<<landmarks_within_sensor_range[lmark].y<<endl; 
	// }
	for(int i = 0; i < particles.size(); ++i){

		cout<<"PARTICLE "<<i<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl; 
		
		// convert all observations from local to global frame 
		std::vector<LandmarkObs> transformed_observations; 
		for(int j = 0; j < observations.size(); ++j){
			LandmarkObs transformed_observation; 
			transformed_observation.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta); 
			transformed_observation.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
			transformed_observation.id = observations[j].id; 
			transformed_observations.push_back(transformed_observation);
		}
		// cout<<"number of transformed observations"<<transformed_observations.size()<<endl; 
		
		// // make list of particles within sensor sensor_range
		std::vector<LandmarkObs> landmarks_within_sensor_range; 
		for(int k = 0; k < map_landmarks.landmark_list.size(); ++k){
			// a landmark from the list
			Map::single_landmark_s lm = map_landmarks.landmark_list[k];
			
			// computing the distance between each particle and each landmark
			double d= dist(particles[i].x, particles[i].y, lm.x_f, lm.y_f);

			// evaluating if the distance is within sensor range
			if(d < sensor_range){
				LandmarkObs close_landmark; 
				close_landmark.x = lm.x_f; 
				close_landmark.y = lm.y_f; 
				close_landmark.id = lm.id_i; 
				landmarks_within_sensor_range.push_back(close_landmark); 
			}
		}

		//  // this is a quick check to see how many landmarks each particle sees
		//  // this can turn up zero
		cout<<"landmarks within sensor range"<<landmarks_within_sensor_range.size()<<endl; 
		for(int lmark = 0; lmark <landmarks_within_sensor_range.size(); ++lmark ){
			cout<<"LANDMARKS IN RANGE:"<<endl; 
			cout<<landmarks_within_sensor_range[lmark].id<<"/"<<landmarks_within_sensor_range[lmark].x<<"/"<<landmarks_within_sensor_range[lmark].y<<endl; 
		}
		// // dataAssociation: this puts the index of predicted_landmarks nearest to each transformed_observations
		// // in the ID field of the transformed_observations element
		
		cout<<"DATA ASSOCOIATION"<<endl ; 
		dataAssociation(landmarks_within_sensor_range, transformed_observations); 


		// // loop through all transformed_observations , use the saved index of the ID to find the associated 
		// // landmark and compute gaussian
		
		// cout<<"GAUSS time"<<transformed_observations.size()<<endl; 
		double particle_probability = 1.0; 
		for(int l = 0; l < transformed_observations.size(); ++l){

			// int id = transformed_observations[l].id -1;  //associated landmark ID 
			// cout<<"transformed observations ID"<<transformed_observations[l].id<<endl; 
			// Map::single_landmark_s associated_landmark = map_landmarks.landmark_list[id];

			// this is the gaussian calculation
			// when I wsa printing out the associations the shortest distance was always one off -- so there is a correction in the map landmark index. I am still not sure why this is
			double c = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);
			// double distance  = dist(transformed_observations[l].x, transformed_observations[l].y, map_landmarks.landmark_list[id].x_f, map_landmarks.landmark_list[id].y_f);
			double a = pow(transformed_observations[l].x - map_landmarks.landmark_list[transformed_observations[l].id -1].x_f, 2) / (2* pow(std_landmark[0], 2)); 
			double b = pow(transformed_observations[l].y - map_landmarks.landmark_list[transformed_observations[l].id -1].y_f, 2) / (2* pow(std_landmark[1], 2));			
			double d =  a + b; 
			double observation_probability = c * exp(-d);
			particle_probability *= observation_probability;  
			
		
			// cout<<"dist"<<distance<<endl; 
			cout<<"a"<<a<<endl;
			cout<<"b"<<b<<endl;
			cout<<"d"<<d<<endl;
			cout<<"gaussian_probability"<<particle_probability <<endl;

		}
		// this updating the particle with the particle weight 
		particles[i].weight =  particle_probability ; 
		// updating the weight vector with the new weight. this is what is used in the resampling wheel below
		weights[i] =  particle_probability ;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<Particle> resampled_particles; 
	double beta = 0.0; 
	double max_weight = *max_element(weights.begin(), weights.end()); 
	default_random_engine gen;
	std::uniform_real_distribution<> uniform_distribution(0, 1);
	int index = int(uniform_distribution(gen)) * (num_particles );

	for(int i = 0; i < num_particles; ++i){
		beta += uniform_distribution(gen) * 2 * max_weight; 
		while(beta > weights[index]){
			beta -= weights[index]; 
			index = (index+1) % num_particles;
		}
		resampled_particles.push_back(particles[index]); 
	}
	particles = resampled_particles; 

	cout<<"resample FINSISHED"<<endl; 
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}