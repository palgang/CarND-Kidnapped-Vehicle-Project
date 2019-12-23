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
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  std::default_random_engine gen;
  num_particles = 100;  // TODO: Set the number of particles
  is_initialized = true;

    // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std[0]);
  
  // These lines create normal distributions for y and theta
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[3]);

  for (int i = 0; i < num_particles; ++i) {
    Particle part;
    part.id = i;
    part.x = dist_x(gen);
    part.y = dist_y(gen);
    part.theta = dist_theta(gen);
    part.weight = 1;

    particles.push_back(part);  
    
  }

    cout << "----------INIT---------------" << endl;
    for (int i = 0; i < particles.size(); ++i) {
      cout << "Particle ID: "<< particles[i].id << endl;
      cout << "Particle x: "<< particles[i].x << endl;
      cout << "Particle y: "<< particles[i].y << endl;
      cout << "Particle theta: "<< particles[i].theta << endl;
      cout << "Particle weight: "<< particles[i].weight << endl;
      cout << "Particle sense: "<< endl;
      for (int j = 0; j < particles[i].sense_x.size(); ++j) {
        cout << "Particle sense_x: "<< particles[i].sense_x[j] <<" Particle sense_y: "<< particles[i].sense_y[j]  << endl;
      }
    }
    cout << "---------- END INIT---------------" << endl;


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
    //cout << "current_x: "<< current_x << " current_y: "<< current_y<< " current_theta: "<< current_theta<< endl;
    //cout << "yaw_rate: " << yaw_rate << endl;
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

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = pred_theta;
    
  }
      cout << "----------PREDICTION---------------" << endl;
      //cout << "delta_t: "<< delta_t << endl;
    //for (int i = 0; i < particles.size(); ++i) {
      //cout << "Particle ID: "<< particles[i].id << endl;
      //cout << "Particle x: "<< particles[i].x << endl;
      //cout << "Particle y: "<< particles[i].y << endl;
      //cout << "Particle theta: "<< particles[i].theta << endl;
      //cout << "Particle weight: "<< particles[i].weight << endl;
      /*cout << "Particle sense: "<< endl;
      for (int j = 0; j < particles[i].sense_x.size(); ++j) {
        cout << "Particle sense_x: "<< particles[i].sense_x[j] <<" Particle sense_y: "<< particles[i].sense_y[j]  << endl;
      }*/
    //}
    cout << "---------- END PREDICTION---------------" << endl;

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
  //cout << "----------dataAssociation---------------" << endl;
  for(int i = 0; i<observations.size();++i){
    double min_distance = 100000000000;
    for(int j = 0;j<predicted.size();++j){
      double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if(distance < min_distance){
        observations[i].id = predicted[j].id;
        min_distance = distance;
        
        /*cout << "observation x: "<< observations[i].x << "observation y: "<< observations[i].y << endl;
        cout << "distance: "<< distance << endl;
        cout << "min distance: "<< min_distance << endl;
        cout << "associated landmark id: "<< observations[i].id << endl;
        cout << "landmark x: "<< predicted[j].x << "landmark y: "<< predicted[j].y << endl;*/
        
      }
    }    
  }


  /*for(int i = 0;i<observations.size(); ++i){
        cout << "observation x: "<< observations[i].x << "observation y: "<< observations[i].y << endl;
        cout << "associated landmark id: "<< observations[i].id << endl;
        //cout << "landmark x: "<< predicted[j].x << "landmark y: "<< predicted[j].y << endl;
  }
  cout << "---------- END dataAssociation---------------" << endl;*/

}
double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
  //cout << ">>>>>>>>>>>>>>>gauss_norm: "<< gauss_norm << endl;

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  //cout << ">>>>>>>>>>>>>>>exponent: "<< exponent << endl;
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
  //cout << ">>>>>>>>>>>>>>>exp(-exponent): "<< exp(-exponent) << endl;
  //cout << ">>>>>>>>>>>>>>>weight: "<< weight << endl;
    
  return weight;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  vector<LandmarkObs>landmarks;
  for(int i = 0; i < map_landmarks.landmark_list.size();++i){
         LandmarkObs lmo;
          lmo.x = map_landmarks.landmark_list[i].x_f;
          lmo.y = map_landmarks.landmark_list[i].y_f;
          lmo.id = map_landmarks.landmark_list[i].id_i;
          landmarks.push_back(lmo);    
  }

 vector<LandmarkObs>obs_map;
  //declare sum:
	//float sum_weights = 0.0f;
 for(int i=0;i<particles.size();++i){
   //particles[i].sense_x.clear();
   //particles[i].sense_y.clear();
   //particles[i].associations.clear();
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
          particles[i].sense_x.clear();
          particles[i].sense_y.clear();
          particles[i].associations.clear();
          particles[i].sense_x.push_back(x_obs_map);
          particles[i].sense_y.push_back(y_obs_map);

          LandmarkObs lmo;
          lmo.x = x_obs_map;
          lmo.y = y_obs_map;
          obs_map.clear();
          obs_map.push_back(lmo);
          dataAssociation(landmarks,obs_map);
          //cout << ">>>>>>>>>>>>>>>obs_map.size(): "<< obs_map.size() << endl;
          particles[i].associations.push_back(obs_map[0].id);
          double weight_obs = multiv_prob(std_landmark[0], std_landmark[1], x_obs_map, y_obs_map,
                   map_landmarks.landmark_list[obs_map[0].id-1].x_f, map_landmarks.landmark_list[obs_map[0].id-1].y_f);
          //cout << ">>>>>>>>>>>>>>>obs_map[0].id: "<< obs_map[0].id << endl;
          /*cout << ">>>>>>>>>>>>>>>std_landmark[0]: "<< std_landmark[0] << endl;
          cout << ">>>>>>>>>>>>>>>std_landmark[1]: "<< std_landmark[1] << endl;*/
          /*cout << ">>>>>>>>>>>>>>>particles[i].x: "<< particles[i].x << endl;
          cout << ">>>>>>>>>>>>>>>particles[i].y: "<< particles[i].y << endl;
          cout << ">>>>>>>>>>>>>>>particles[i].theta: "<< particles[i].theta << endl;
          cout << ">>>>>>>>>>>>>>>observations[j].x: "<< observations[j].x << endl;
          cout << ">>>>>>>>>>>>>>>observations[j].y: "<< observations[j].y << endl;
          
          cout << ">>>>>>>>>>>>>>>x_obs_map: "<< x_obs_map << endl;
          cout << ">>>>>>>>>>>>>>>y_obs_map: "<< y_obs_map << endl;
          cout << ">>>>>>>>>>>>>>>map_landmarks.landmark_list[obs_map[0].id].x_f: "<< map_landmarks.landmark_list[obs_map[0].id-1].x_f << endl;
          cout << ">>>>>>>>>>>>>>>map_landmarks.landmark_list[obs_map[0].id].y_f: "<< map_landmarks.landmark_list[obs_map[0].id-1].y_f << endl;
          cout << ">>>>>>>>>>>>>>>weight_obs: "<< weight_obs << endl;*/
          //if(updated_weight == -1){
            //updated_weight = weight_obs;
          //}else{
            updated_weight *= weight_obs; 
            //particles[i].weight *=weight_obs;<-- weight update
          //}
        }
        particles[i].weight = updated_weight;
        //sum_weights += updated_weight;
   }
        //if(updated_weight > 0){
          //particles[i].weight = updated_weight; <-- weight update
          //sum_weights += updated_weight;
        //}else{
          //particles[i].weight = 1.0;
          //sum_weights = 1.0;
        //}
  }

  // normalization
  float sum_weights = 0.0f;
  for(int i = 0; i<particles.size();++i){
    sum_weights += particles[i].weight;
  }
  for(int i = 0;i<particles.size();++i){
    particles[i].weight = particles[i].weight/sum_weights;
  }

  /*for(int i = 0; i<particles.size(); ++i){
    if(sum_weights > 0){
      particles[i].weight = particles[i].weight/sum_weights;
    //weights[i] = particles[i].weight;
    }else{
      particles[i].weight = particles[i].weight/sum_weights;
      cout << "sum_weights <=0: "<< sum_weights << endl;
      cout << "particles[i].weight: "<< particles[i].weight << endl;
      //cout << "observatsions.size() "<< observations.size() << endl;
      //cout << "particles.size() "<< particles.size() << endl;
    }
  
  }*/

    //cout << "----------UPDATEWEIGHT---------------" << endl;
    //cout << "sensor_range: "<< sensor_range << endl;
    //for (int i = 0; i < particles.size(); ++i) {
      //cout << "Particle ID: "<< particles[i].id << endl;
      //cout << "Particle x: "<< particles[i].x << endl;
      //cout << "Particle y: "<< particles[i].y << endl;
      //cout << "Particle theta: "<< particles[i].theta << endl;
      //cout << "Particle weight: "<< particles[i].weight << endl;
      /*cout << "Particle sense: "<< endl;
      for (int j = 0; j < particles[i].sense_x.size(); ++j) {
        cout << "Particle sense_x: "<< particles[i].sense_x[j] <<" Particle sense_y: "<< particles[i].sense_y[j]  << endl;
        cout << "Association: "<< particles[i].associations[j] <<" Landmark x: "<< map_landmarks.landmark_list[particles[i].associations[j]].x_f<<" Landmark y: "<< map_landmarks.landmark_list[particles[i].associations[j]].y_f  << endl;
      }*/
    //}
    //cout << "---------- END UPDATEWEIGHT---------------" << endl;

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  cout << "---------- Resample---------------" << endl;
  vector<double> w;
  for(int i=0;i<particles.size();++i){
    w.push_back(particles[i].weight);
    //cout << w[i] << endl;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  // Create the distribution with weights
  std::discrete_distribution<> d(w.begin(), w.end());

  vector<Particle> sampled_particles;
  for(int n = 0;n<particles.size();++n){
    int ind = d(gen);
    //cout << "ind: " << ind << endl;
    //cout << "particles[ind].id: " << particles[ind].id << endl;
    //sampled_particles.push_back(particles[d(gen)]);
    sampled_particles.push_back(particles[ind]);
  }
  particles = sampled_particles;
  cout << "Resampled particles" << endl;
  for(int i = 0;i<particles.size();++i){
    cout << "x: "<<particles[i].x << " y: " << particles[i].y << " theta: " << particles[i].theta << endl;
  }
  cout << "---------- End Resample---------------" << endl;
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
  cout << "getAssociations" <<endl;
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  cout << "END getAssociations" <<endl;
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  cout << "getSenseCoord" <<endl;
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
  cout << "END getSenseCoord" <<endl;
  return s;
}