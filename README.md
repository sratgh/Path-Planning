# Writeup

## Warmup
To begin with I started implementing waypoints that direct the car to go in a straight line (first task from the classroom) as well as in a circle (second part from the classroom).
Once I understood how that worked I moved on with the detailed presentation (Q&A video) from the classroom and implemented the shown code after having understood what that code did and how exactly it worked.

The next challenge was to have the car follow the middle lane, where it starts in the simulation.

A couple of important things to understand first.

At the end of the onMessage() function in main.cpp the two vectors
```c++
vector<double> next_x_vals;
vector<double> next_y_vals;
```
need to be filled with values in x, y coordinates, that the car will follow.

Since x, y coordinates are needed, but for generating waypoints, splines, etc. it is much easier to use Freenet coordinates, the getXY() function from the helper section is very handy.

The idea then is to generate waypoints with a for loop and increment the s variable each time, while d stays constant (we do want to stay in our lane at first).

After having done that getXY() can be used to transform s, d Freenet coordinates to x, y cartesian coordinates. At the end the values are pushed to the two vectors and the program can be run.


```c++
double dist_inc = 0.5;
for (int i = 0; i < 50; i++ )
{
  double next_s = car_s + (i+1) * dist_inc;
  double next_d = 2+4;
  vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  next_x_vals.push_back(xy[0]);
  next_y_vals.push_back(xy[1]);
}
```

A few more important things to understand about this peace of code are:
- by creating waypoints with specific s and d values, and the fact that this function is called every 0.02s the velocity with which the car drives to these waypoints will be implicitly given
- therefore the distance between the waypoints divided by 0.02s is the velocity the car will go at (ideal controller)
- the lanes are numbered from left to right with 0, 1 and 2

## Improvements and spline calculation
Having achieved that the car was following the lane it started it, further steps needed to be addressed. These were:
- The car jumped from 0 to the 50mph speed limit within the first waypoint, which created too much acceleration and jerk
- The car just drove into other vehicles without breaking
- Additionally the acceleration and jerk was exceeded in curves, since the given waypoints had sharp edges and the yaw-rate was exeeded

To adress the last point it is important to make use of smooth, differentiable lines i.e. splines that go through given waypoints while considering differentiability at the beginning and end of the spline

I took the nice spline.h library from the given link in the classroom and further followed the detailed instruction from the Q&A video.

 Most important take-aways for me were the fact that it makes a lot of sense to push all the previous waypoints, that the car has not yet been approaching can just be appended to the next_xy_vals vectors. This way, we do not need to calculate a whole new trajectory every iteration (0.02s). For creating new waypoints and the spline in between it is actually enough to take at least two previous waypoints (important for having a differentiable continuation of the spline) and then a few further in the distance. In freenet coordinates we do need to make the d value dependent on a lane variable for having the spline follow a desired lane change. See the following code

 ```c++
 vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
 // The lane variable in the d-parameter takes into account that we might want to change lanes
 ```

 As mentioned before, velocity is implicitly defined by the distance between the waypoints. In this case we have two previous waypoints and a few further in the distance. What we still need now, is to fill in all the values in between. The way we can do this, is to actually divide a straight line into as much in between waypoints as we need for our desired velocity and then calculate the corresponding spline value.

 ```c++
 double target_x = 30.0; // divide x-axis into 30 waypoints
 double target_y = s(target_x); // get corresponding spline y-value
 double target_dist = sqrt(target_x*target_x+target_y*target_y); // calculate distance between waypoints

 double x_add_on = 0;

 for(int i = 1; i <= 50-previous_path_x.size(); i++)
 {
   double n = target_dist / (0.02 * ref_vel / 2.24); // Calculate number of points necessary for the desired speed
   //...
 }
 ```

After having run this state of the implementation I could see that the car was smoothly following its starting lane.


## Driving strategy
Next task was to decelerate when a car is in front and to change lanes as soon as the desired lane is free from other cars.

The most simple way I could think about achieving this was to implement a decision based approach.

The following cases were taken into account:
- Another car is in front (distance <30m) --> flag that car is too close
- Another car is on the right lane (within a range of 15m before us and 10m behind us) --> set flag that there is traffic on the right lane
- Another car is on the left lane (within a range of 15m before us and 10m behind us) --> set flag that there is traffic on the left lane

```c++
if(d < (2+4*lane+2) && d > (2+4*lane-2) && (check_car_s > car_s) && ((check_car_s-car_s) < 30)) // If other car is in our lane before us
{
    too_close = true;
}
if((lane == 0 && d > 4 && d < 8 && check_car_s-car_s < 15 && check_car_s-car_s > -10) || \
        (lane == 1 && d > 8 && d < 12 && check_car_s-car_s < 15 && check_car_s-car_s > -10))  // If other car is on our right side
{
  car_on_right_side = true;
}
if((lane == 2 && d > 4 && d < 8 && check_car_s-car_s < 15 && check_car_s-car_s > -10) || \
        (lane == 1 && d > 0 && d < 4 && check_car_s-car_s < 15 && check_car_s-car_s > -10)) // If other car is on our left side
{
  car_on_left_side = true;
}

}
```

Now we set the most basic flags for implementing a driving strategy through the traffic.
The following code shows how this has been implemented.

```c++
// slowly change to desired velocity while not exceeding < 10m/s<2 acceleration
if (too_close)
{
  ref_vel -= .300;
}
// slowly increase up to our speed limit of just beneath 50mph
else if (ref_vel < 49.5)
{
  ref_vel += .300;
}
// Change to the right lane if there is no traffic and we have a car in front of us and we are not already on the right-most lane
if(too_close && car_on_right_side == false && lane < 2 && lane_change_started == false)
{
  lane++;
  lane_change_started = true;
}
// Change to the left lane if there is no traffic and we have a car in front of us and we are not already on the left-most lane
else if(too_close && car_on_left_side == false && lane > 0 && lane_change_started == false)
{
  lane--;
  lane_change_started = true;
}

// Since we do not want to have the car change lanes every 0.02 seconds, a counter has been implemented that prohibits lane  changes
// for a few seconds
if (lane_change_started)
{
  timer++;
}
// Set back counter and allow lane changes again
if (timer > 100)
{
  lane_change_started = false;
  timer = 0;
}
```

With this implementation I could achieve more than seven miles in the simulation.


# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

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
