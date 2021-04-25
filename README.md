# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./images/simulator.PNG "Simulation screen"
[image2]: ./images/highway_path_f.GIF "Result"


### to safely navigate around a virtual highway with other traffic
- Drive +-10 MPH of the 50 MPH speed limit.
- Be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. 
- Go as close as possible to the 50 MPH speed limit
- Pass slower traffic when possible
- Avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times
- Go from one lane to another, if necessary.
- The car should be able to make one complete loop around the 6946m highway.
- total acceleration is less than 10 m/s^2 and jerk is less than 10 m/s^3.

## Reflection
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

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

## Reflection
As shown in the screen below, I explain how we passed each rubric point to avoid “accidents” classified by the simulator on tracks that were 4.32 miles or longer.
![alt text][image1]

- Checking all vehicles on sensor fusion data and making high-level decisions about whether to change lanes or change speeds.
- There is a car in front of me, so my car have to change lanes or slow down. 
- There is a vehicle obstructing the right or left lane so I cannot change lanes and I have to slow down.
- I also don't do double lane changes.
- The distance from the target lane to the nearest car in front of my car is greater than the distance to the car in front of us in my lane.

### Construct a spline to sample a new trajectory point

- The first control points are the last two points on the unhandled/remaining path returned by the simulator.
- Three more control points are also added 30m, 60m and 90m ahead of the last point in the Frenet space.

### Spline sampling to return the trajectory points the path should follow

- The spline is first returned from the simulator and filled with points that have not yet been processed.
- The spline is always sampled up to 30m ahead
- The number of points sampled is determined by the reference speed, so it covers the exact distance between each trajectory point the car passes through every 0.2 seconds.

### Driving according to the speed limit
If the speed limit is not reached, increase by the reference speed (0.224 mph).

### Maximum acceleration and jerk compliance
In order to observe the maximum acceleration and jerk, the speed is increased and decreased (0.224mph) in small increments.

### Collision avoidance

- Make sure there is no traffic in the right or left lane
- It also does not double change lanes.
- The neighbouring car's s coordinates must be within my car +/- preferred buffer if not this means that I have enough space to make a transition.
- car_s - 30.0 < o_s (other's speen) < car_s + 30.0

### My driving
![alt text][image2]

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









