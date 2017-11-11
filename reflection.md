# **Model Predicitve Control** 

**Model Predicitve Control Project**

The goals / steps of this project are the following:
* Implement the MPC controller as taught in lesson 19 "Model Predicitve Control"
* Tune the MPC controller parameters such that the vehicle safely drives through the simulated race track
* Summarize the results with a written report


[//]: # (Video References)

[videoP]:   ./P_video.mp4   "P Controller Video"
[videoPD]:  ./PD_video.mp4  "PD Controller Video"
[videoPID]: ./PID_video.mp4 "PID Controller Video"


## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

## Implementation of Model Predictive Controller

#### The model

I used the kinematic model as presented in chapter 12 "Motion Models" of the lecture. Here, the used state variables are the *(x, y)*-coordinates, the vehicle orientation *theta* and the vehicle longitudinal speed *v*. In order to incorporate the actuators (steering angle *delta* and acceleration *a*) I combined these equations with the version presented in chapter 18 "Vehicle Models" (i.e., the definition of the temporal derivative of theta).

For *delta != 0*, the resulting system model update equations read:

*x_f	= x_0 + (L/delta_0) * [sin(theta_0 + (v_0*delta_0/L)*dt) - sin(theta_0)]*

*y_f = y_0 + (L/delta_0) * [cos(theta_0) - cos(theta_0 + (v_0*delta_0/L)*dt)]*

*theta_f = theta_0 + (v_0*delta_0/L)*dt*

*v_f = v_0 + a_0 * dt*

For *delta = 0*, the resulting system model update equations read:

*x_f	= x_0 + v_0 * cos(theta_0) * dt*

*y_f = y_0 + v_0 * sin(theta_0) * dt*

*theta_f = theta_0*

*v_f = v_0 + a_0 * dt*

In addition, I used the trajectory error update equations (cross track error *cte* and orientation error *thetae*) as presented in chapter 18 "Vehicle Models":

*cte_f = cte_0 + v_0 * sin(thetae) * dt*

*thetae_f = thetae_0 + (v_0*delta_0/L)*dt*

#### Timestep Length and Elapsed Duration (*N* & *dt*)

In order to cope with the latency of the system, which seems to be roughly equal to the time for which the calculated control values will acually be applied, I chose dt equal to the latency, i.e., *dt* = 0.1 seconds. This value seemed reasonable as a first try and worked out fine.
As number of timestamps I first chose *N* = 20 and later decreased this value to 10. A control time horizon of 2 seconds seemed (and seems) reasonable to me, but due to the fact that the vehicle may drive through sharp turns the trajectory points may turn by approximately 90 degrees in that time frame. As a result, the matched polynomial will contain a very high derivative, which again leads to instabilities (loops) in the model predictive control. During my testing I therefore gradually decreased this value to 10 in order to always obtain a stable reference and predicted trajectory for my chosen vehicle speed.

#### Polynomial Fitting and MPC Preprocessing

In order to obtain a reference trajectory as input to the model predictive control algorithm, I first transformed the received waypoints to the vehicle coordinate system. Next I fitted a 3rd order polynomial through these points. I then used this polynomial as reference trajectory for the model predictive control algorithm.

Next I projected the vehicle state by the latency to the future using the currently received actuator values and the vehicle model described above. I used these values as input to the model predictive control algorithm.

#### Model Predictive Control with Latency

I decided to cope with the latency of 100 milliseconds in 2 ways: Firstly, I defined the model predictive control time increment according to the latency. Like this, the time frame for which the control values will actually be applied to the actuators will be equal to the time frame used in their computation by the optimization algorithm. Secondly, I projected the vehicle state by the latency to the future, such that the optimization algorithm would optimize the actuator values for the vehicle state at which they will be applied. Both measures together with a reasonable choice of the cost parameters of the model predictive control algorithm resulted in a control strategy which allows the vehicle to drive smoothly through the race track.

As a reasonable choice of cost parameters of the optimization objective function I scaled the cost values according to the magnitude of the respective variables: I scaled all errors with a factor of 1.0, the suqared difference to the reference speed with a factor of 0.01 (as the speed is of magnitude 10 and 0.01 * 10*10 = 1.0), the squared magnitude of the actuator values for *delta* 4.0 times as much as the squared magnitude of the actuator values for *a* (because the angle values are constrained to be within [-0.436332, 0.436332] whereas the acceleration values are constrained to be within [-1.0, 1.0] and this is roughly a factor of 2.0).