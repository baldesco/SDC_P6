# **Project 6: Kidnapped Vehicle**

This document presents the work developed for the 6th project of the SDC Nanodegree.

## Project Introduction
Your vehicle has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

Using the available information, the objective of this project is to complete the code provided to implement a particle filter that will estimate the vehicle's position (*x*, *y*) and heading (*theta*) with small error. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step, the filter will also get observation and control data.

Some of the steps that are needed to complete this project are:

* Randomly initialize a defined number of particles, setting their first position based on (noisy) GPS estimates of *x*, *y*, and *theta*.
* Use the velocity and yaw rate of the vehicle to predict its new position after a time *delta_t*. To achieve this, a bicycle motion model is used. This prediction step takes into account the uncertainty in the vehicle's controls.
* Update the relative weight of each of the filter's particles, according to the combined probabilities of the vehicle's observations. In this step it is necessary to be careful with the vehicle and the particles coordinate systems.
* Resample the filter's particles, in order to keep those with a larger relative weight. These particles are the ones that are more likely to have *x*, *y* and *theta* values closer to the actual vehicle.

## Results
This project has two main criteria to determine if the implementation of the particle filter is successful:

1. Accuracy: The particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.
2. Performance: The particle filter should complete execution within the time of 100 seconds.

If everything is ok, the simulator outputs the following message:

```
Success! Your particle filter passed!
```

Below, I show a video of the performance of the filter, showing that it complies with all requirements.

<img src="media/result.gif" width="600">

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Communication protocol with the simulator
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT:** values provided by the simulator to the c++ program
// sense noisy position data from the simulator
["sense_x"]
["sense_y"]
["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state
["previous_velocity"]
["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values
["sense_observations_x"]
["sense_observations_y"]

**OUTPUT:** values provided by the c++ program to the simulator
// best particle values used for calculating the error evaluation
["best_particle_x"]
["best_particle_y"]
["best_particle_theta"]

// for respective (x,y) sensed positions ID label
["best_particle_associations"]

// for respective (x,y) sensed positions
["best_particle_sense_x"] <= list of sensed x positions
["best_particle_sense_y"] <= list of sensed y positions

# Implementing the Particle Filter
The directory structure of this repository is as follows:
```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.
