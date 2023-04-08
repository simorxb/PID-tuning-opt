import numpy as np
from scipy.optimize import minimize, LinearConstraint, OptimizeResult
import matplotlib.pyplot as plt

class PID:

    """ This class implements a PID controller.
    """

    def __init__(self, Kp, Ki, Kd, Kaw, T_C, T, max, min, max_rate):
        self.Kp = Kp                # Proportional gain
        self.Ki = Ki                # Integral gain
        self.Kd = Kd                # Derivative gain
        self.Kaw = Kaw              # Anti-windup gain
        self.T_C = T_C              # Time constant for derivative filtering
        self.T = T                  # Time step
        self.max = max              # Maximum command
        self.min = min              # Minimum command
        self.max_rate = max_rate    # Maximum rate of change of the command
        self.integral = 0           # Integral term
        self.err_prev = 0           # Previous error
        self.deriv_prev = 0         # Previous derivative
        self.command_sat_prev = 0   # Previous saturated command
        self.command_prev = 0       # Previous command
        self.command_sat = 0        # Current saturated command

    def Step(self, measurement, setpoint):
        """ Execute a step of the PID controller.

        Inputs:
            measurement: current measurement of the process variable
            setpoint: desired value of the process variable
        """

        # Calculate error
        err = setpoint - measurement

        # Update integral term with anti-windup
        self.integral += self.Ki*err*self.T + self.Kaw*(self.command_sat_prev - self.command_prev)*self.T
        
        # Calculate filtered derivative
        deriv_filt = (err - self.err_prev + self.T_C*self.deriv_prev)/(self.T + self.T_C)
        self.err_prev = err
        self.deriv_prev = deriv_filt

        # Calculate command using PID equation
        command = self.Kp*err + self.integral + self.Kd*deriv_filt

        # Store previous command
        self.command_prev = command

        # Saturate command
        if command > self.max:
            self.command_sat = self.max
        elif command < self.min:
            self.command_sat = self.min
        else:
            self.command_sat = command

        # Apply rate limiter
        if self.command_sat > self.command_sat_prev + self.max_rate*self.T:
            self.command_sat = self.command_sat_prev + self.max_rate*self.T
        elif self.command_sat < self.command_sat_prev - self.max_rate*self.T:
            self.command_sat = self.command_sat_prev - self.max_rate*self.T

        # Store previous saturated command
        self.command_sat_prev = self.command_sat

class Object:

    """ This class represents a 1D object, subject to a force F, with mass m, 
        viscous damping coefficient k, F_max/F_min forces, and time step T. 
    """

    def __init__(self, m, k, F_max, F_min, T):
        self.m = m                  # Mass of the object
        self.k = k                  # Damping constant
        self.F_max = F_max          # Max force applied to the object
        self.F_min = F_min          # Min force applied to the object
        self.T = T                  # Time step
        self.v = 0                  # Velocity of the object
        self.z = 0                  # Position of the object

    def Step(self, F):

        """ Update the position of the object based on the applied force F.
        """

        # Saturate input force
        if F > self.F_max:
            F_sat = self.F_max

        elif F < self.F_min:
            F_sat = self.F_min
        else:
            F_sat = F

        # Calculate the derivative dv/dt using the input force and the object's velocity and properties
        dv_dt = (F_sat - self.k*self.v)/self.m

        # Update the velocity and position of the object by integrating the derivative using the time step T
        self.v += dv_dt*self.T
        self.z += self.v*self.T

def Simulation(x, time_step, end_time, m, k, F_max, F_min, max_rate):
    
    """ Simulate the PID control of a 1D object with given parameters.
    
        Returns:
        (t, stp, z, command): arrays of time, setpoints, positions, and commands
    """

    length = round(end_time/time_step)

    t = np.zeros(length)
    stp = np.zeros(length)
    z = np.zeros(length)
    command = np.zeros(length)

    [Kp, Ki, Kd, Kaw, T_C] = x

    # Initialize PID controller
    pid = PID(Kp, Ki, Kd, Kaw, T_C, time_step, F_max, F_min, max_rate)

    # Initialize object with given parameters
    obj = Object(m, k, F_max, F_min, time_step)

    # Iterate through time steps
    for idx in range(0, length):
        t[idx] = idx*time_step
        # Set setpoint
        stp[idx] = 500
        
        # Execute the control loop
        z[idx] = obj.z
        pid.Step(z[idx], stp[idx])
        command[idx] = pid.command_sat
        obj.Step(command[idx])  
    
    return (t, stp, z, command)

def Cost(x, time_step, end_time, m, k, F_max, F_min, max_rate, We, Wu):
    """ Calculate the cost function for a given set of PID parameters.

        Inputs:
        x: PID parameters [Kp, Ki, Kd, Kaw, T_C]
        We: weight on position error
        Wu: weight on control effort

        Returns:
        cost: scalar value representing the total cost
    """

    (t, stp, z, command) = Simulation(x, time_step, end_time, m, k, F_max, F_min, max_rate)

    cost = np.sum(np.square(stp - z))*We + np.sum(np.square(command))*Wu

    return cost

# -------- Configuration --------

# Simulation parameters

time_step = 0.1
end_time = 80
length = round(end_time/time_step)

# Object parameters

m = 10
k = 0.5
F_max = 100
F_min = -100

# Controller parameters

max_rate = 40

# Optimization weights for cost function

We = [1, 30, 1]
Wu = [1, 1, 30]

# Initialize arrays for storing results

t = np.zeros((length, len(We)))
stp = np.zeros((length, len(We)))
command = np.zeros((length, len(We)))
z = np.zeros((length, len(We)))
result = []

# Perform minimization for each combination of We and Wu weights

for idx in range(0, len(We)):
    bounds = ((0, None), (0, None), (0, None), (0, None), (0, None))
    r = minimize(Cost, [1, 0.1, 5, 0.1, 1], args=(time_step, end_time, m, k, F_max, F_min, max_rate, We[idx], Wu[idx]), bounds=bounds)
    result.append(r)

    # Print optimization results

    print("We = " + "{:.3g}".format(We[idx]) + " Wu = " + "{:.3g}".format(Wu[idx])  + " Kp = " + "{:.3g}".format(result[idx].x[0])   
          + " Ki = " + "{:.3g}".format(result[idx].x[1])  + " Kd = " + "{:.3g}".format(result[idx].x[2])  
          + " Kaw = " + "{:.3g}".format(result[idx].x[3])  + " T_c = " + "{:.3g}".format(result[idx].x[4]))
    print("Success: " + str(r.success))

    # # Run simulation with optimized parameters
    (t[:, idx], stp[:, idx], z[:, idx], command[:, idx]) = Simulation(r.x, time_step, end_time, m, k, F_max, F_min, max_rate)

# Plot position response

plt.subplot(2, 1, 1)
for idx in range(0, len(We)):
    plt.plot(t[:,idx], z[:,idx], label="Response - We = " + "{:.3g}".format(We[idx]) + " Wu = " + "{:.3g}".format(Wu[idx])  
             + " - Kp = " + "{:.3g}".format(result[idx].x[0])   + ", Ki = " + "{:.3g}".format(result[idx].x[1])  + ", Kd = " + "{:.3g}".format(result[idx].x[2])  
             + ", Kaw = " + "{:.3g}".format(result[idx].x[3])  + ", T_c = " + "{:.3g}".format(result[idx].x[4]))
plt.plot(t[:,0], stp[:,0], '--', label="Setpoint [m]")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()

# Plot command force

plt.subplot(2, 1, 2)
for idx in range(0, len(We)):
    plt.plot(t[:,idx], command[:,idx], label="Command - We = " + "{:.3g}".format(We[idx]) + " Wu = " + "{:.3g}".format(Wu[idx])  
             + " - Kp = " + "{:.3g}".format(result[idx].x[0])   + ", Ki = " + "{:.3g}".format(result[idx].x[1])  + ", Kd = " + "{:.3g}".format(result[idx].x[2])  
             + ", Kaw = " + "{:.3g}".format(result[idx].x[3])  + ", T_c = " + "{:.3g}".format(result[idx].x[4]))
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()
plt.grid()

# Display the plots

plt.show()