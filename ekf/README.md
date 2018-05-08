[//]: # (Image References) 
[ekf_tracking]: ./ekf_tracking.png

This project implements an extended Kalman filter in C++. 

Input data consisting of laser measurements (given directly as
x and y positions, with some known uncertainty) and radar
measurements (given as radius, angle, and radial velocity 
relative to some fixed measurement site, with some known uncertainty)
are combined with a motion model to track a vehicle with much better
accuracy than the individual measurements alone allow.

The code presented here is designed to work with the
Udacity term 2 simulation executable, and so cannot be run standalone.
However, here's some example output.  

![Tracking car with EKF][ekf_tracking]

Red circles are lidar measurements.

Blue circles are radar measurements (position markers inferred from radius and angle; 
the also-supplied radial velocity measurements are not shown).

Green markers are the car's position as estimated by the Kalman filter.
It's clear that the Kalman filter does a good job of tracking the car's 
position with significantly reduced noise.


