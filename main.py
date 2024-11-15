import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as Math

# PID Controller Function
def PID(P, I, D, error, rate, integral):
    return (P * error + I * integral + D * rate)

# Ascent Rate Function
def ascent_rate(time):
    if time < 42:
        rate = ((161 * (time ** 4)) / 100000000) - ((33 * (time ** 2)) / 5000) + ((7 * time) / 50)
        if rate < 0:
            rate /= 2.5
    else:
        rate = -0.1
    return rate

# Force Vectors Calculation
def force_vectors(angles, thrust):
    Xangle = angles[0]
    Yangle = angles[1]

    alpha = np.radians(Xangle + 90)
    beta = np.radians(Yangle)

    # Initial velocity components
    x_vector = Math.cos(alpha) * Math.cos(beta) * thrust
    z_vector = Math.sin(alpha) * Math.cos(beta) * thrust
    y_vector = Math.sin(beta) * thrust

    return x_vector, y_vector, z_vector

def atmosphere(height):
    start_density = 1.225  # approximate density of atmosphere at sea level, kg/m^3
    # the atmospheric thinning is modeled as an exponential decay
    alt_density = start_density * (2.71828 ** (-0.00012 * (height + 250)))
    return alt_density

def Drag_vectors(angles, Velocities, Side_Area, Frontal_Area, Cd, Altitude):
    Xangle = angles[0]
    Yangle = angles[1]

    alpha = np.radians(Xangle + 90)
    beta = np.radians(Yangle)

    # Initial velocity components
    x_area = Math.cos(alpha) * Math.cos(beta) * Frontal_Area + (1 - Math.cos(alpha) * Math.cos(beta)) * Side_Area
    z_area = Math.sin(alpha) * Math.cos(beta) * Frontal_Area + (1 - Math.sin(alpha) * Math.cos(beta)) * Side_Area
    y_area = Math.sin(beta) * Frontal_Area + (1 - Math.sin(beta)) * Side_Area

    Density = atmosphere(Altitude)

    x_vector = 0.5 * Cd * x_area * (Velocities[0] ** 2) * Density
    y_vector = 0.5 * Cd * y_area * (Velocities[1] ** 2) * Density
    z_vector = 0.5 * Cd * z_area * (Velocities[2] ** 2) * Density

    if Velocities[0] > 0:
        x_vector *= -1
    if Velocities[1] > 0:
        y_vector *= -1
    if Velocities[2] > 0:
        z_vector *= -1

    return x_vector, y_vector, z_vector

# Change in Rotational Velocity Calculation
def change_in_rotational_velocity(force, distance, mass, length, time_step, radius):
    torque = force * distance
    moment_of_inertia = (mass * (length ** 2) / 12) + (mass * (radius ** 2) / 4)
    angular_acceleration = torque / moment_of_inertia
    change_in_rotational_velocity = angular_acceleration * time_step
    return change_in_rotational_velocity

# Plot Initialization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-20, 20])
ax.set_ylim([-20, 20])
ax.set_zlim([0, 20])

fig2 = plt.figure()
x_angle_plot = fig2.add_subplot(111)
x_angle_plot.set_ylim([-90, 90])

fig3 = plt.figure()
y_angle_plot = fig3.add_subplot(111)
y_angle_plot.set_ylim([-90, 90])

fig4 = plt.figure()
altitude_plot = fig4.add_subplot(111)

figure2, axis2 = plt.subplots(2, 2)
figure, axis = plt.subplots(2, 2)

# Simulation Parameters
step = 0.0001  # Time step in seconds
time = 0  # Initial time in seconds
burn_time = 50  # Motor burn time in seconds
PID_Control = True
Throttle_control = True

# Rocket Physical Properties
mass = 3  # Mass in kg
length = 1.1  # Length in meters
radius = 0.1  # Radius in meters
cg_to_gimbal_distance = 0.6  # Distance from center of gravity to gimbal in meters
Side_Area = 0.25 # Side Area in m^2
Frontal_Area = 0.0314 # Frontal Area in m^2
Cd = 0.95 # Drag Coefficient

# Initial Conditions
x = 0  # Initial x position in meters
y = 0  # Initial y position in meters
z = 0  # Initial z position in meters
x_velocity = 0  # Initial x velocity in m/s
y_velocity = 0  # Initial y velocity in m/s
z_velocity = 0  # Initial z velocity in m/s
rotational_velocity = [0, 0]  # Initial rotational velocities in rad/s
vector_angles = [0, 0]  # Initial vector angles in degrees
vector_angles_setpoint = [0, 0]  # Initial vector angles set-point in degrees
acc_vector = []  # Acceleration vector
gravity_acceleration = -9.81  # Gravity acceleration in m/s^2
launch_angle_x_degrees = -20  # Launch angle in degrees for x-axis
launch_angle_y_degrees = 0  # Launch angle in degrees for y-axis

# Thrust and Gimbal Parameters
max_thrust = 35  # Maximum thrust in Newtons
throttle_percentage = 100  # Throttle percentage
gimbal_rate = 30  # Gimbal rate in deg/s
x_gimbal_limit = 20  # Gimbal limit for x-axis in degrees
y_gimbal_limit = 20  # Gimbal limit for y-axis in degrees

# PID Controller Gains
x_axis_P = 0.5
x_axis_I = 0.02
x_axis_D = 0.3
y_axis_P = 0.5
y_axis_I = 0.02
y_axis_D = 0.3
throttle_P = 390
throttle_I = 0
throttle_D = 12
lateral_compensation = 0

# PID Controller Integrals
x_integral = 0
y_integral = 0
throttle_integral = 0

# Plotting Parameters
plot_frequency = 2000
plot_frequency_tvc = 300
Plot_Counter = 0
Plot_Counter2 = 0
Plot_Counter3 = 0

# Ground Check
has_left_ground = False

# Simulation Loop
while z > -0.01:
    x_angle_setpoint = x_velocity * lateral_compensation
    y_angle_setpoint = y_velocity * -lateral_compensation

    if z > 0:
        has_left_ground = True
    else:
        has_left_ground = False
        x_velocity = 0

    Plot_Counter += 1
    Plot_Counter2 += 1
    Plot_Counter3 += 1
    x_integral += launch_angle_x_degrees * step
    y_integral += launch_angle_y_degrees * step
    throttle_integral += (ascent_rate(time) - z_velocity) * step

    for i in range(2):
        if vector_angles[i] < vector_angles_setpoint[i]:
            vector_angles[i] += gimbal_rate * step
        elif vector_angles[i] > vector_angles_setpoint[i]:
            vector_angles[i] -= gimbal_rate * step

    if PID_Control:
        vector_angles_setpoint[0] = PID(x_axis_P, x_axis_I, x_axis_D, launch_angle_x_degrees - x_angle_setpoint, rotational_velocity[0], x_integral)
        vector_angles_setpoint[1] = -PID(y_axis_P, y_axis_I, y_axis_D, launch_angle_y_degrees - y_angle_setpoint, rotational_velocity[1], y_integral)

    vector_angles[0] = max(min(vector_angles[0], x_gimbal_limit), -x_gimbal_limit)
    vector_angles[1] = max(min(vector_angles[1], y_gimbal_limit), -y_gimbal_limit)

    thrust_vectors = force_vectors(vector_angles, max_thrust * throttle_percentage / 100)
    rotational_velocity[0] += math.degrees(change_in_rotational_velocity(thrust_vectors[0], cg_to_gimbal_distance, mass, length, step, radius))
    rotational_velocity[1] += math.degrees(change_in_rotational_velocity(thrust_vectors[1], cg_to_gimbal_distance, mass, length, step, radius))

    launch_angle_x_degrees += rotational_velocity[0] * step
    launch_angle_y_degrees += rotational_velocity[1] * step


    if Throttle_control:
        throttle_percentage = PID(throttle_P, throttle_I, throttle_D, ascent_rate(time) - z_velocity, acc_vector[2] / mass * step, throttle_integral)

    if throttle_percentage > 100:
        throttle_percentage = 100
    elif throttle_percentage < 0:
        throttle_percentage = 0
    burn_time -= throttle_percentage / 100 * step



    if burn_time > 0:
        acc_vector = force_vectors([launch_angle_x_degrees, launch_angle_y_degrees], thrust_vectors[2])
        if Plot_Counter == plot_frequency:
            ax.plot([x], [y], [z], 'bo')
            Plot_Counter = 0
    else:
        acc_vector = [0, 0, 0]
        if Plot_Counter == plot_frequency:
            ax.plot([x], [y], [z], 'ro')
            Plot_Counter = 0

    if Plot_Counter3 == plot_frequency_tvc:
        axis[0,0].plot([time], [throttle_percentage], 'bo')
        axis[0,1].plot([time], [vector_angles[0]], 'bo')
        axis[1,0].plot([time], [vector_angles[1]], 'bo')
        axis[1,1].plot([time], z_velocity, 'bo')
        Plot_Counter3 = 0

    if Plot_Counter2 == plot_frequency:
        x_angle_plot.plot([time], [launch_angle_x_degrees], 'bo')
        y_angle_plot.plot([time], [launch_angle_y_degrees], 'bo')
        altitude_plot.plot([time], [z], 'bo')

        axis2[0,0].plot([time], [x_velocity], 'bo')
        axis2[0,1].plot([time], [y_velocity], 'bo')
        axis2[1,0].plot([time], [x], 'bo')
        axis2[1,1].plot([time], [y], 'bo')
        Plot_Counter2 = 0

    z_velocity += acc_vector[2] / mass * step
    x_velocity += acc_vector[0] / mass * step
    y_velocity += acc_vector[1] / mass * step

    drag_vector = Drag_vectors([launch_angle_x_degrees,launch_angle_y_degrees], [x_velocity, y_velocity, z_velocity], Side_Area, Frontal_Area, Cd, z)

    x_velocity += drag_vector[0] / mass * step
    y_velocity += drag_vector[1] / mass * step
    z_velocity += drag_vector[2] / mass * step


    if has_left_ground:
        z_velocity += gravity_acceleration * step

    x += x_velocity * step
    y += y_velocity * step
    z += z_velocity * step

    time += step

# Plot Labels and Titles
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Rocket Trajectory')

x_angle_plot.set_xlabel('Time')
x_angle_plot.set_ylabel('X Angle')

y_angle_plot.set_xlabel('Time')
y_angle_plot.set_ylabel('Y Angle')

altitude_plot.set_xlabel('Time')
altitude_plot.set_ylabel('Altitude (m)')

axis[0,0].set_xlabel('Time')
axis[0,0].set_ylabel('Throttle')

axis[0,1].set_xlabel('Time')
axis[0,1].set_ylabel('X Vector Angle')

axis[1,0].set_xlabel('Time')
axis[1,0].set_ylabel('Y Vector Angle')

axis[1,1].set_xlabel('Time')
axis[1,1].set_ylabel('Z Velocity')

axis2[0,0].set_xlabel('Time')
axis2[0,0].set_ylabel('X Velocity')

axis2[0,1].set_xlabel('Time')
axis2[0,1].set_ylabel('Y Velocity')

axis2[1,0].set_xlabel('Time')
axis2[1,0].set_ylabel('X Position')

axis2[1,1].set_xlabel('Time')
axis2[1,1].set_ylabel('Y Position')

plt.show()