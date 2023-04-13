# PID parameter optimisation

Optimisation of PID parameters implemented in Python, using scipy.optimize.minimize.

Code created to support a Linkedin post. Link to the original post: https://www.linkedin.com/posts/simone-bertoni-control-eng_pid-parameter-optimisation-activity-7050406931177328640-soQh?utm_source=share&utm_medium=member_desktop

Follow me on Linkedin! https://www.linkedin.com/in/simone-bertoni-control-eng/

Looking for a little-known way to tune a PID?

Python can help. Curious? 👇

If you know the model of the plant (linear, non-linear, acausal, doesn't really matter) you can use this approach.

Let's see how!

The equation of a PID controller with filtered derivative and back-calculation anti-windup is:

U(s) = K_p E(s) + (K_i E(s) + K_aw (U_sat(s) - U(s)))\*1/s + K_d E(s) \* s/(T_c s + 1)

where U_sat(s) is the command U(s) saturated (always necessary to match the actuator's capability) and E(s) = R(s) - Y(s).

The idea is to find the optimal combination of the PID parameters (\(K_p, K_i, K_d, K_{aw}, T_c\)) that minimises a cost function:

J = sum[W_e\*(r_i - y_i)^2 + W_u\*u_sat_i^2]

where:

✅ r: setpoint
✅ y: controlled variable
✅ u_sat: saturated command
✅ W_e: weighting coefficient related to the control error
✅ W_u: weighting coefficient related to u_sat

The choice of W_e and W_u determines the compromise between minimising the error and minimising the control effort:

✅ W_e > W_u: minimising the error is more important than minimising the control effort

✅ W_e < W_u: minimising the control effort is more important than minimising the error

The example in the carousel uses a plant that I have shown before:

An object of mass m = 10 kg, sliding on a surface where it's subject to viscous friction of coefficient k = 0.5 N\*s/m, pushed by a force F.

The dynamic equation is:

d^2z/dt^2 m = F - k \* dz/dt

The Python code (link to GitHub in the comments) consists of:

✅ A "PID" class that implements a PID controller with filtered derivative and back-calculation anti-windup, with a method "Step" that executes a step of the controller.

✅ An "Object" class that represents the plant described before, with a method "Step" that updates the position of the object based on the applied force F.

✅ A function "Simulation" that runs a simulation of 80 seconds where the PID controls the object's position following a step setpoint of 500 m.

✅ A function "Cost" that runs the function "Simulation" with a set of PID parameters and computes the cost function J

✅ A main code that runs the optimisation 3 times with different combinations of W_e and W_u and plots the different performances

Here are the 3 different configurations and their results:

Configuration 1

W_e = 1, W_u = 1

Result: Kp = 1.65, Ki = 0.228, Kd = 4.87, Kaw = 0.301, T_c = 0.506

Configuration 2

W_e = 30, W_u = 1

Result: Kp = 1.07, Ki = 0.0119, Kd = 4.99, Kaw = 0.0123, T_c = 1.01

Configuration 3

W_e = 1, W_u = 30

Result: Kp = 1.04, Ki = 0.0941, Kd = 4.99, Kaw = 0.176, T_c = 0.981

If you enjoyed this follow me for more tips on control and embedded software engineering.

Hit the 🔔 on my profile to get a notification for all my new posts.

What do you want to know? Ask in the comments!

#controlsystems #embeddedsystems #softwareengineering #embeddedsoftware #coding #controltheory #python #PID
