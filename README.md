# Path Planning Project
In this project I designed and developed the software and the algorithms of a path planner for an autonomous driving car.
The code is written in C++ and the program communicates with a simulator provided by Udacity where there are several vehicles driving on a three lanes road.
This work is part of the Self-Driving Car Engineer Nanodegree Program at Udacity.

---

[//]: # (Image References)
[simulator]: ./img/5miles.png
[UML]: ./img/UML-diagram.png
[arch]: ./img/software-architecture.png
[video]: ./img/40secs_7fps.gif
[p]: ./img/p.gif
[q]: ./img/q.gif
[s_star]: ./img/s_star.gif
[q_s_star]: ./img/q_s_star.gif
[qs_s_star]: ./img/qd_s_star.gif
[condition1]: ./img/condition1.gif
[condition2]: ./img/condition2.gif
[theta]: ./img/theta.gif
[d]: ./img/d.gif
[p_q]: ./img/p_q.gif
[eq3]: ./img/eq3.gif


![Simulator][simulator]

## Results

The algorithms have been tested in a simulator provided by Udacity for about 10 minutes without incidents, driving more than 6 miles.
[A realization of this has been uploaded to YouTube in this link,](https://youtu.be/a_IoRniavFc)
and here you can see a short example:

![GIF Example][video]


## Software Architecture

In the following picture we can see a high-level diagram of the modules required and the way they relate to each other:

![Software Architecture][arch]

Basically, the simulator sends sensor data of the ego vehicle and the surrounding environment. With this in mind, I model the road with the kinematic state of each vehicle sensed and predict how this is going to change in the near future using a kinematic model. Based on that, the behavior model loops over several goal points and generates trajectories. For each trajectory, it calculates a cost, which represents how safe it is (including max-jerk penalization, max-speed, collision detection, etc). The best one is sent back to the simulator. Most of the core logic is coded in the method `Behavior::UpdateState()`, in the file `behavior.cpp`.


## UML

Here we can see the classes that are implemented to accomplish the mission.

![UML diagram][UML]

## Map

A series of waypoints are also provided which allow as to create a global map of the road. 

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Coordinate Transform

In order to convert from Cartesian to Frenet coordinate system and vice-versa, a new algorithm is designed:

Let 
![p eq][p]
be the point in cartesian coordinates, and 
![q eq][q]
the parametric formula of the curve.
Also, let 
![][s_star]
be the parameter for which 
![][q_s_star]
is closest to 
**p**
, then the
following condition must be satisfied:

![][condition1]

which means that the distance between 
**p** 
and 
![][q_s_star]
is perpendicular to 
![][qd_s_star]
where 
![][qd_s_star]
is actually the tangent vector of the curve at point 
![][s_star].
Let 
![][theta].
be the curve angle at point 
*s*
, then

![][condition2]

[//]: # (see -link to paper-)

The condition (1) could have multiple roots in a global domain, but only one is
the closest point to 
**p**. 
However, it can be shown that if 
![][s_star]
is unique and we constrain to a local domain around
the closest waypoint to 
**p**
, then (1) is sufficient to find a unique 
![][s_star].

Having said that, in order to implement the conversion from Cartesian to Frenet I first find the closest waypoint, get the *s* value and start moving in a gradient-decent fashion until eq. (1) is zero (within some defined tolerance). Then the *d* coordinate is the distance between the point and the curve 

![][d]

However, this won't give us the sign correctly. So what I do is to calculate it as the 3rd component of the vector product between 
![][p_q]
and the oriented tangent, eq. (2). So the formula is:

![][eq3]

In conclusion, the algorithm goes like this:
```
Find the closest waypoint to the given point. 
Get the *s* value for that waypoint 
Do:
  s <- s + the result of eq. (1)
  Calculate the tangent in that point
  Calculate eq. (1)
While eq. (1) is not zero
d <- eq. (3)
```
In order to get q(s) I store splines with the x and y coordinates as a function of the s parameter based on the provided waypoints.

The Frenet to Cartesian conversion is really straight forward. Using the given *s* parameter I calculate the cartesian point along the waypoint track:

![q eq][q]

and then add the d value along the direction of the **d** vector for that point (which is also provided for each waypoint).

The code that implements all this can be found inside `map.cpp`, in the functions `ToFrenet()` and `ToCartesian()`.

## Achievements

**The car drives according to the speed limit**
In order to achieve this, the algorithm discards trajectories that are going to exceed the speed limit in the future, so it picks a trajectory with less acceleration or overall speed.

**Max Acceleration and Jerk are not Exceeded**
The acceleration increments implemented by the behavior model were designed carefully so that neither the acceleration nor the jerk are exceeded of 10 m/s^2 and 10 m/s^3 respectively.

**Car does not have collisions.**
One of the most important cost functions implemented in the trajectory generation algorithm is `TrajectoryCost::DetectCollision()`. This function iterates over the candidate trajectory predicting the future state of the road for each step. Internally, to detect collisions I designed an algorithm that simplifies each vehicle to an ellipse, so it is easier and faster to calculate.

**The car stays in its lane, except for the time between changing lanes.**
This is one of the hardest points to achieve, because there are times when several trajectories have almost the same cost so it loops between CHANGE LANE and KEEP LANE states. When that happens, the ego vehicle ends up going straight in the line that divides lanes. In order to overcome this problem I added a functionality to allow the ego vehicle to complete the CHANGE LANE state if it has decided to start it (bypassing the possibility to KEEP LANE). This way if it decides to change lanes, it won't stop till it is in the next lane, unless a new vehicle appears and the change suddenly becomes risky.

**The car is able to change lanes**
As you can see in the GIF and in the 
[YouTube video](https://youtu.be/a_IoRniavFc)
the ego vehicle changes lanes very well when there is a vehicle in front of him going slower and there is no traffic in the next lane


--------------

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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
