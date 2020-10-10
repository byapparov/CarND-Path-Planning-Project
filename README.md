# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Reflection

To meet specification of the project I have broken the problem into three parts:

* Assesing the safe speed of the car based on the predictions from sensor fusion about other cars (`target_lane_safe_speed()` in `trajectory_cost.h`);
* Generation of smooth trajectories that don't vialate speed, acceleration and jerk constraints (`Vehicle::compute_trajectory()`);
* Selection of trajectories based on the cost functions that prevent colision, maximise speed and keep the car in the central lane if it is not taking over other cars (`trajectory_cost.h`).

## Example

Here is an example of how `Vehcile()` class can be used to drive the car:

```cpp
#include "vehicle.h"

Vehicle vehicle(
  1, // initial lane of the car
  0.0, // initial speed of the car
  47.5 / 2.23, // target speed, aka speed limit
  map_waypoints_s, 
  map_waypoints_x, 
  map_waypoints_y
);

// within the message update
h.onMessage(...) {

  // update the state of the car with information from sensors
  vehicle.Update( 
    car_x, 
    car_y, 
    deg2rad(car_yaw), 
    car_speed,
    car_s, car_d,
    previous_path_x, previous_path_y,
    end_path_s, end_path_d);
  
  // update the state of the car to the new optimal state based on the trajectory costs
  vehicle.SwitchState(sensor_fusion); 
  
  // get current best trajectory cost of the car
  vector<vector<double>> trajectory = vehicle.Trajectory(); 
  
  
  // trajectory points can be used to update the controls of the car
  // to follow optimal chosed trajectory
  next_x_vals = trajectory[0];
  next_y_vals = trajectory[1];

}
```

## Model Documentation

All code files for this project can be found in the `src` folder.

### Code files

Code of the model consist of the following modules:

|File|Type|Description|
|--|---|--------------|
|vehicle.h|class| `Vehicle` class manages trajectory of the car and contains data structure to store key information about the car's state|
|trajectory_cost.h|functions|Functions to estimate different components of trajectory cost|
|sensor_fusion.h|functions|Function to predict state of surounding vehicles based on data from sensor fusion|
|vehicle_states|functions|Function that generates next possible states given the current state of the car|
|map.h|functions|Helper functions to support unit conversions, space transformations etc.|
|helpers.h|functions|Helper functions to support Frenet to XY transformations of coordinates|

### External single file libraries

|File|Type|Description|
|--|---|--------------|
|spline.h|class|Contains `Spline` class that was used in this project to generate acceptable trajectories|
|catch.hpp|framework|Catch2 framework for unit testing of cpp code|

### Trajectory Generation

In this soluion, I have generated trajectories using `Spline` class from the corresponding [library](https://kluge.in-chemnitz.de/opensource/spline/).

The process of trajectory generation involved following steps:

* Conversion of the target and reference Frenet coordinates into the map coordinates using `getXY()` functions from `helpers.h`.
* Conversion of the trajectory target coordinates into coordinates of the car. These transformations can be found in the `map.h` file in functions `GlobalCoordinates()` and `LocalCoordinates()`;
* Incrementing trajectory with deltas that would not break the limits on speed, acceleration and jerk. To acheieve this I have used `Vehicle()` class as the state machine of the car's acceleration which was updated with every new point of trajectory. This allowed me to acheive the maximum performance in terms of acceleration within the constraints of the project. 

### Testing

Unit tests for the project were implemented using [catch2](https://github.com/catchorg/Catch2).

All the tests can be found in `tset_helpers.cpp` and executed 


### Future improvements

Current implementation demonstrated acceptable performance on the road with the car being able to accelarate and slow down depending on the speed and position of the car ahead. It also makes good lane transitions while avoiding colisions.

I have used a very simple approach to estimate if another car is blocking a given trajectory, which does not calculate possitions of the cars in all time points of the trajectory. This means that in rear conditions a car moving sideways at speed could cause a colision. I have not observed this situation in the simulator, but it would be safer if the full lengths of trajectories were checked. 

One other imrovement could be associated with probabalistic approach to predictions of other cars' positions, which could make selection of trajectories safer.

In my solution, I have only implemented three states for the car - Keep Lane (KL),  Lane Change Left (LCL) and Lane Change Right (LCR). This was enough to achieve objectives of the project. To make navigation of the car optimal, solution with more states and dynamic programming optimiser can be developed. This would improve car navigation in certain situations, where for example it is blocked from the immidate lane transition by another car.


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

