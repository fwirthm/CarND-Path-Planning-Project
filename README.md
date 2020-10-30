# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


### Model Documentation
The model which steers the vehicle along the track is constructed as follows:

First of all the algorithm analyses all surrounding objects which are identified by the sensors. As a results it identifies the closest preceeding vehicles on the lane of the ego vehicle and on its left and right lane , respectively. To check on which lanes the detected objects are, the frenet coordinates are used. Besides, it is checked whether lane changes to the left and right lane are safe. This condition is fulfilled if there is no vehicle on the respective target lane in a range of 30m in front and behind the ego vehicle. 

After analyzing the ego vehicles surrounding its own path is planned. If there is no preceeding vehicle in sensor range on the current lane, the vehicle accelerates. If there is a preceeding vehicle several cases can occur. The first case is that the preceeding vehicle is slower than the ego vehicle and closer than 30m. In that case, a lane change would be a good idea. But in order to first enlarge the gap size we are slightly deaccelarating. If the vehicle should be even closer to the ego (<10m) the acceleration is set to a maximum deacceleration value. This can be understood as a kind of safety breaking. This can be quite helpful if a vehicle spontaneously cuts in on our lane. If the vehicle is driving in front of us but more than 30m away, we can check if we should prepare a lane change if this has not already be done. 

If we are already about to prepare a lane change, first of all we check if the ego velocity is in a certain range (25-35 mph) that ensures that the lane change does not happen with to large jerks and accelerations and that it is still safe. Changing the lane very slowly can in fact be dangerous as our safety distance could not be sufficent. If that is not fulfilled we speed up or slow down. If we are in the desired velocity range we check if the lane change to the desired lane is still safe. If so, the lane change is scheduled and the acceleration is fixed to o. A condition esnures, that not more than one lane changes is scheduled within a time window of 2 seconds. 

To check whether or not a lane change should be prepared the following logic is applied: If the preceeding car is more than 70m away or driving only slightly below our desired speed, no action needs to be taken. We can simply wait untill we are closer to the vehicle or simply follow it. Otherwise it depends on which lane we are which lane change options we have. If we are on the left most or right most lane respectively only one lane change direction is possible. Should we in contrast be on the middle lane a both lane change directions would be possible in the first place. To decide which direction has to be favored we look at the velocities of the preceeding vehicles on that lanes and decide for the one with the faster vehicle in front. If one or both lanes should not be occupied the choice is eased. In all cases the velocity of the preceeding vehicle on the the target lane should be larger than our current preecing vehicles - otherwise a lane change is not reasonable.

An additional timing condition ensures that the vehicle acceleration is hold at a avlue of 0 no matter what other conditions tell the vehicle, if a lane change maneuver was started less than 2 seconds ago. On startup or low speeds this condition is not necessary.

A final condition checks, that the maximum allowed speed is not exceeded. Within our implementation we set the maximum speed to 43.5 mph. This ensures both that the project requirement of driving approximately at 50+-10 mph is fulfilled and that we can react fast enough for example on vehicles cutting in onto our lane.

Finally, our planned path is smoothed using spline interpolation to ensure low jerks. The trajectory generated that way is then used to steer the vehicle.


#### Reflection
The described shortcoming could be fixed as follows:
- Sometimes other cars change its lane very spontaneously and cut in onto the ego lane. The current implementation spline interpolation implementation only replanes as less points as possible. In the described case, the systems reaction time can therefore be to large. To fix that it could be helpful to replan the whole trajectory in that cases. 




### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

