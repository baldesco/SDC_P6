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
#include <random>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;
using std::pow;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to first position
   * (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
   * 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  // Set the number of particles
  num_particles = 100;  //optimal number according to https://knowledge.udacity.com/questions/29851

  // Get the standard deviations for x, y, and theta
  double std_x, std_y, std_theta;
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];

  // Create a normal (Gaussian) distribution for x. y and theta
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  // Random number generator
  default_random_engine gen;

  // Initialize the particles in a loop
  for (int i = 0; i < num_particles; ++i){
    double sample_x, sample_y, sample_theta;
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);

    Particle p;
    p.id = i;
    p.x = sample_x;
    p.y = sample_y;
    p.theta = sample_theta;
    p.weight = 1;

    particles.push_back(p);
    weights.push_back(1);
  }
  // The particle filter is now marked as initialized
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  // Get the location and heading sigmas from std_pos
  double std_x, std_y, std_theta;
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];
  // Random number generator
  default_random_engine gen;

  // Loop over the particles to predict the motion of each of them
  for (int i = 0; i < num_particles; ++i){
    Particle p = particles[i];
    float xf, yf, thetaf;
    // The motion model equations depend on the yaw rate (zero or non-zero)
    if (yaw_rate==0){
      thetaf = p.theta;
      xf = p.x + velocity*delta_t*cos(thetaf);
      yf = p.y + velocity*delta_t*sin(thetaf);
    } else{
      thetaf = p.theta + yaw_rate*delta_t;
      xf = p.x + (velocity/yaw_rate) * (sin(thetaf) - sin(p.theta));
      yf = p.y + (velocity/yaw_rate) * (cos(p.theta) - cos(thetaf));
    }
    // Normal distributions to add gaussian noise to the predictions
    normal_distribution<double> dist_x(xf, std_x);
    normal_distribution<double> dist_y(yf, std_y);
    normal_distribution<double> dist_theta(thetaf, std_theta);
    // Update the position and heading of the particle
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each observed measurement 
   * and assign the observed measurement to this particular landmark.
   */
  // Loop over the observations
  for (unsigned int i = 0; i < observations.size(); ++i){
    // Set a very large initial minimal distance
    double distance,min_dist;
    min_dist = 200000000;
    // Loop over all the predictions, to compare each one of them with each observation
    for (unsigned int j = 0; j < predicted.size(); ++j){
      // Calculate distance between observation and prediction, and compare to min_dist
      distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
      if (distance < min_dist){
        // update minimun distance
        min_dist = distance;
        // associate prediction 'j' to observation 'i'
        observations[i].id = predicted[j].id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian
   * distribution. You can read more about this distribution here:
   * https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * 
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   * Your particles are located according to the MAP'S coordinate system.
   * You will need to transform between the two systems. Keep in mind that
   * this transformation requires both rotation AND translation (but no scaling).
   * The following is a good resource for the theory:
   * https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   * and the following is a good resource for the actual equation to implement
   * (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  double x_map, y_map, x_part, y_part, x_obs, y_obs, theta, sig_x, sig_y;

  // The standard deviation of x and y is the same for all particles
  sig_x = std_landmark[0];
  sig_y = std_landmark[1];

  // 0. Initialize the sum of the particles weights
  double weights_sum = 0;

  // Loop over each particle to calculate its weight
  for (int k=0; k < num_particles; ++k){
    // 1. Get location and heading of the particle
    x_part = particles[k].x;
    y_part = particles[k].y;
    theta = particles[k].theta;

    // 2. Transform the coordinates of the observations to map coordinates
    vector<LandmarkObs> map_observations;

    for (unsigned int j=0; j < observations.size(); ++j){
      x_obs = observations[j].x;
      y_obs = observations[j].y;

      x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

      LandmarkObs new_obs;
      new_obs.x = x_map;
      new_obs.y = y_map;
      new_obs.id = observations[j].id;

      map_observations.push_back(new_obs);
    }

    // 3. Create a vector to put in the predicted map landmark locations
    vector<LandmarkObs> predicted_landmarks;

    // Loop over the real landmarks, and find if they are within the particle's sensor range
    for (unsigned int i=0; i<map_landmarks.landmark_list.size(); ++i){
      double landmark_distance;
      landmark_distance = dist(x_part,y_part,map_landmarks.landmark_list[i].x_f,map_landmarks.landmark_list[i].y_f);
      if (landmark_distance <= sensor_range){
        LandmarkObs lm;
        lm.x = map_landmarks.landmark_list[i].x_f;
        lm.y = map_landmarks.landmark_list[i].y_f;
        lm.id = map_landmarks.landmark_list[i].id_i;
        predicted_landmarks.push_back(lm);
      }
    }
    // 4. For each of the observations (in map coordinates) associate the nearest prediction
    dataAssociation(predicted_landmarks, map_observations);

    // 5. Calculate the probability of each observation, and the particles's total weight
    double particle_weight = 1.0;
    // Loop over each observarion
    for (unsigned int i=0; i<map_observations.size(); ++i){
      double x_obs, y_obs, mu_x, mu_y;
      // x and y position of the observation
      x_obs = map_observations[i].x;
      y_obs = map_observations[i].y;
      // Find the prediction associated with this observation, to get mu_x and mu_y
      for (unsigned int j=0; j<predicted_landmarks.size(); j++){
        if(map_observations[i].id == predicted_landmarks[j].id){
          mu_x = predicted_landmarks[j].x;
          mu_y = predicted_landmarks[j].y;
          break;
        }
      }
      // Calculate the probability of this observation
      double single_prob = multi_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);
      // Update the particle's total weight by multiplying this probability
      particle_weight *= single_prob;
    }
    // 6. Update the particle's weight
    particles[k].weight = particle_weight;
    weights_sum += particle_weight;
  }
  // 7. Finally, normalize the weights of the particles so they all add to one
  for (int k=0; k<num_particles; ++k){
    particles[k].weight /= weights_sum;
    weights[k] = particles[k].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional
   * to their weight.
   * 
   * NOTE: You may find std::discrete_distribution helpful here.
   * http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   // Define a discrete distribution
   std::discrete_distribution<int> weighted_dist(weights.begin(),weights.end());
   // Random number generator
   default_random_engine gen;

   // Create vector to allocate particles after resampling
   vector<Particle> resampled;
   // Perform the resampling N times, where N is the number of particles
   for (int i =0; i<num_particles; ++i){
     int index = weighted_dist(gen);
     resampled.push_back(particles[index]);
     // reset the weight of the ith particle
     weights[i] = 1;
   }
   // Replace the particles for the resampled ones
   particles = resampled;
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
