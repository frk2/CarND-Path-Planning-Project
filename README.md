# CarND-Path-Planning-Project
Extermely interesting project - I feel my solution is too simplistic but it seems to satisfy the rubric. More info on how to build lane change path planning cost functions would be greatly appreciated!

### General driving and lane changes
This was done pretty much exactly as outlined in the QA. The only difference is that I try to make smoother turns by extending the spline generation to 50,100,150 instead of 30,60,90. Also my car follow is smoother since I try to match the speed of the car in front of us.

Lane changing is accomplished simply by changing the last 3 spline points.

### Behavior Planning
There is a simplistic cost function based on the earlier behavior planner exercise. The planner only executes if we are being slowed down by a car. We figure out average lane speeds of the cars closest to us and then see if a change to the left or the right lane would cause a collision. In case its not safe, a high cost of 1.0 is assigned so that this lane can never be chosen. Lane changing itself has a cost attached to it, so in case we are all surrounded by cars the tie breaker is to always keep in lane.

The planner outputs 'KL', 'LCL' and 'LCR' which do what you would expect. It also outputs 'PLCL' and 'PLCR' which currently do nothing and mean 'KL'

The car seems to drive fine with these very simplistic plans :)

### Future work
- The car following code needs to be made into a PID controller of sorts.
- getXY needs to be ported to use spline
- Some emergency braking needs to happen if cars jump in front of you

### known limitations
There seem to be drunk robocars on the road and sometimes one would just literally move in front of my car. My car brakes too slow so it ends up rear-ending this drunk driver :) Should there be a emergency braking system built into the car? Am guessing a PID controller would help here too.
