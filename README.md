# PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


In this project, my goal is to write a PID controller with hyperparameters optimized for a test track.

## The Project

The steps to accomplish the goal in this project are the following:

* Properly initialize the PID
* Update error vector at each timestep
* Convert PID output to steering input
* Fine tune hyperparameters

## PID

[PID controller](https://en.wikipedia.org/wiki/PID_controller) (Proportional Integral Derivative controller) is a control mechanism widely used in several industrial applications.

One of it's many uses in robotics is to smoothen the robot's movement inputs in a way that it allows the robot to reach it's objective state with as little noise as possible.

The output of a PID depends on three hyperparameters, one for each term of our controller (P-I-D). These hyperparameters are specific to each application, which in our case is driving a car smoothly around a test track.

### Proportional term

> The proportional term produces an output value that is proportional to the current error value. The proportional response can be adjusted by multiplying the error by a constant Kp, called the proportional gain constant.
The proportional term is given by:

>P_out = Kp * crosstrack_error

>A high proportional gain results in a large change in the output for a given change in the error. If the proportional gain is too high, the system can become unstable (see the section on loop tuning). In contrast, a small gain results in a small output response to a large input error, and a less responsive or less sensitive controller. If the proportional gain is too low, the control action may be too small when responding to system disturbances. Tuning theory and industrial practice indicate that the proportional term should contribute the bulk of the output change

The chart below shows how different values of Kp would impact the PID output.
![alt text][image1]
Source: wikipedia

### Integral term

>The contribution from the integral term is proportional to both the magnitude of the error and the duration of the error. The integral in a PID controller is the sum of the instantaneous error over time and gives the accumulated offset that should have been corrected previously. The accumulated error is then multiplied by the integral gain (Ki) and added to the controller output.

>The integral term is given by
I_out = Ki * integral_error

The integral error is the integral of the error over the observation period. In our case where the interval between observations is constant, it gets simplified by the sum of all crosstrack errors observed.

>The integral term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure proportional controller. However, since the integral term responds to accumulated errors from the past, it can cause the present value to overshoot the setpoint value (see the section on loop tuning).

The chart below shows how different values of Ki would impact the PID output.
![alt text][image2]
Source: wikipedia

### Differential term

> The derivative of the process error is calculated by determining the slope of the error over time and multiplying this rate of change by the derivative gain Kd. The magnitude of the contribution of the derivative term to the overall control action is termed the derivative gain, Kd.

>The derivative term is given by
D_out = Kd * derivative_error


The chart below shows how different values of Kd would impact the PID output.

![alt text][image3]
Source: wikipedia


## Fine tuning hyperparameters

These hyperparameters can be tuned using several different methods. I decided against spending much time in thinking about the best way to optimize these hyperparameters, and the reason will become obvious in the last section of this analysis.

One of the suggested approaches to tune our hyperparameters is an automated trial and error method that Sebastian Thrun named "Twiddle". It changes each of the hyperparameters one at a time and analyzes the results of these changes. If they are better we keep the new parameters, if they are worse we change them again.

Although not very scientific, the results usually converge to a local minimum that has good results for most applications.

## Final considerations

While PID controllers are extremely important in the industry, this static version is not good enough for a production self-driving car.

The final product we have after optimizing the hyperparameters is a overfitted solution that may very well be useful for a single application, but will fail to react to changes in the environment and mechanical failures of the car - such as heavy winds or a steering bias being introduced after hitting a pothole.

That is the reason why I decided against spending a lot of time in fine-tuning the hyperparameters.

A better approach would be to have an Adaptive PID controller, that could constantly react to changes and ensure a smooth and safe driving condition under any given circumstances.


## References
[1] [PID controller](https://en.wikipedia.org/wiki/PID_controller)


[//]: # (Image References)

[image1]: ./images/Change_with_Kp.jpg "Kp comparison"
[image2]: ./images/Change_with_Ki.png "Ki Comparison"
[image3]: ./images/Change_with_Kd.png "Ki Comparison"
