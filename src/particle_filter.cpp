/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  std::default_random_engine gen;
  num_particles = 100;  //Set the number of particles
  is_initialized = true;

    // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std[0]);
  
  // These lines create normal distributions for y and theta
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[3]);

  // Initialize particles
  for (int i = 0; i < num_particles; ++i) {
    Particle part;
    part.id = i;
    part.x = dist_x(gen); // x + random Gaussian noise
    part.y = dist_y(gen); // y + random Gaussian noise
    part.theta = dist_theta(gen); // theta + random Gaussian noise
    part.weight = 1;

    particles.push_back(part);  
    
  }

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  for(int i = 0;i<particles.size();++i){
    double current_x = particles[i].x;
    double current_y = particles[i].y;
    double current_theta = particles[i].theta;

    double pred_theta;
    double pred_x;
    double pred_y;

    // Different set of equations for yaw_rate equal 0 and yaw_rate not equal 0
    if(yaw_rate==0){
      pred_theta = current_theta;
      pred_x = current_x + velocity*delta_t*cos(current_theta);
      pred_y = current_y + velocity*delta_t*sin(current_theta);
    }else{
      pred_theta = current_theta + yaw_rate*delta_t;
      pred_x = current_x + velocity/yaw_rate*(sin(pred_theta)-sin(current_theta));
      pred_y = current_y + velocity/yaw_rate*(cos(current_theta)-cos(pred_theta));
    }

    // This line creates a normal (Gaussian) distribution for x,y
    normal_distribution<double> dist_x(pred_x, std_pos[0]);
    normal_distribution<double> dist_y(pred_y, std_pos[1]);

    particles[i].x = dist_x(gen); // predicted x + random Gaussian noise
    particles[i].y = dist_y(gen); // predicted y + random Gaussian noise
    particles[i].theta = pred_theta; // predicted theta + random Gaussian noise
    
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for(int i = 0; i<observations.size();++i){
    double min_distance = 100000000000;
    // Find predicted measurement with the smallest distance to landmark
    for(int j = 0;j<predicted.size();++j){
      double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if(distance < min_distance){
        observations[i].id = predicted[j].id;
        min_distance = distance;                
      }
    }    
  }
}

// Calculate weight using Multi-variate Gaussian
double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
  
  return weight;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // Vector of landmarks as observations
  vector<LandmarkObs>landmarks;
  for(int i = 0; i < map_landmarks.landmark_list.size();++i){
         LandmarkObs lmo;
          lmo.x = map_landmarks.landmark_list[i].x_f;
          lmo.y = map_landmarks.landmark_list[i].y_f;
          lmo.id = map_landmarks.landmark_list[i].id_i;
          landmarks.push_back(lmo);    
  }

 vector<LandmarkObs>obs_map;
 for(int i=0;i<particles.size();++i){

   double updated_weight = 1;

   for(int j=0;j<observations.size();++j){

        // transform to map x coordinate
        double x_obs_map;
        x_obs_map = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);

        // transform to map y coordinate
        double y_obs_map;
        y_obs_map = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);

        // check whether observation is within the sensor range
        if(dist(particles[i].x,particles[i].y,x_obs_map,y_obs_map)<=sensor_range){

          // add observations to particle
          particles[i].sense_x.clear();
          particles[i].sense_y.clear();
          particles[i].associations.clear();
          particles[i].sense_x.push_back(x_obs_map);
          particles[i].sense_y.push_back(y_obs_map);

          // add observation to vector of landmark observations
          LandmarkObs lmo;
          lmo.x = x_obs_map;
          lmo.y = y_obs_map;
          obs_map.clear();
          obs_map.push_back(lmo);

          dataAssociation(landmarks,obs_map);

          particles[i].associations.push_back(obs_map[0].id);

          double weight_obs = multiv_prob(std_landmark[0], std_landmark[1], x_obs_map, y_obs_map,
                   map_landmarks.landmark_list[obs_map[0].id-1].x_f, map_landmarks.landmark_list[obs_map[0].id-1].y_f);

          updated_weight *= weight_obs; 
        } // end if in sensor_range
   } // end loop over observations
   particles[i].weight = updated_weight;
  } // end loop over particles

  // normalization
  float sum_weights = 0.0f;
  for(int i = 0; i<particles.size();++i){
    sum_weights += particles[i].weight;
  }
  for(int i = 0;i<particles.size();++i){
    particles[i].weight = particles[i].weight/sum_weights;
  }

}

void ParticleFilter::resample() {
  /**
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<double> w;
  for(int i=0;i<particles.size();++i){
    w.push_back(particles[i].weight);
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  // Create the distribution with weights
  std::discrete_distribution<> d(w.begin(), w.end());

  vector<Particle> sampled_particles;
  for(int n = 0;n<particles.size();++n){
    int ind = d(gen);
    sampled_particles.push_back(particles[ind]);
  }

  particles = sampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}