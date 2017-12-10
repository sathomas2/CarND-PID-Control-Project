# Proportional Integral Derivative Controller for Autonomous Cars
### Overview:
Assuming an autonomous car has accurately perceived its environment with cameras, LIDAR, RADAR, GPS, or other sensors, the problem remains of what it should do with that information. The problem is twofold. What path should it take? And, how should it behave to follow that path? In this project, the problem is simplified because the track in the Udacity simulator is a single lane road with turns that ultimately forms a large loop. Therefore, the car's best path is stay in the center of the lane. In a first attempt to achieve this, I will implement a proportional-integral-derivative (PID) controller to control the steering angle of the car. 

The contents of this repository include:
```
root
|   .gitignore
|   CMakeLists.txt
|   CODEOWNERS
|   README.md
|   cmakepatch.txt
|   install-mac.sh
|   install-ubuntu.sh
|   project_assignment.md
|
|___readme_images
|   |   
|   |   ...
|   
|___src
    |   main.cpp
    |   PID.h
    |   PID.cpp
    |   json.hpp
```

Everything in the root directory, except for this README, was provided by Udacity to install the necessary dependencies. CODEOWNERS is the citation of the author of the starter code. The readme_images directory contains the images in this README. main.cpp connects to the simulator using WebSocketIO and makes the necessary calls to the PID class. PID.cpp contains my implementation of the PID controller as well as a simple solution I devised for controlling the throttle and breaks of an autonomous vehicle.

### PID Controller:
Given a desired set point SP=r(t) and a process variable PV=y(t), I can measure an error term at every time step e(t) as the diffence between them. 
 <figure>
  <img src="readme_images/CTE.png"/>
</figure>
 <p></p>
The PID controller minimizes e(t) by updating a control variable u(t) at each time step. For example, if the SP is the center of the lane, then e(t) is the distance of the car from the lane center or its cross-track error (CTE). By updating u(t) or the steering angle at each time step, the PID controller will minimize the CTE.

As its name suggests, there are three terms to this update, proportional (P), integral (I), and derivative (D), so that:
 <figure>
  <img src="readme_images/PID.png"/>
</figure>
 <p></p>
 The proportional term is the product of the CTE and a parameter that must be tuned, the P-gain (Kp).
 <figure>
  <img src="readme_images/P.png"/>
</figure>
 <p></p>
 The integral term is the product of the integral of the CTE over time and a parameter, the I-gain (Ki).
 <figure>
  <img src="readme_images/I.png"/>
</figure>
 <p></p>
 The derivative term is the product of the derivative of the CTE with respect to time or its rate of change and a parameter, the D-gain (Kd).
 <figure>
  <img src="readme_images/D.png"/>
</figure>
 <p></p>
 Putting this together, we get:
  <figure>
  <img src="readme_images/PIDcomplete.png"/>
</figure>
 <p></p>
For my implementation, see PID.cpp lines 54-79.

### Tuning the Gain Parameters:
### Throttle Control:
### Results:
