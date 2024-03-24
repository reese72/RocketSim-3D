import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as Math

def PID(P, I, D, error, rate, integral):
    return (P * error + I * integral + D * rate)

def ascent_rate(time):
    if time < 47.5:
        rate = ((161 * (time ** 4)) / 100000000) - ((33 * (time ** 2)) / 5000) + ((7 * time) / 50)
    else:
        rate = -0.2
    return rate

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

def change_in_rotational_velocity(force, distance, mass, length, time_step):
    # Calculate the torque as the cross product of the force and distance vectors
    torque = force * distance

    # Calculate the moment of inertia for the rocket
    moment_of_inertia = mass * (length ** 2) / 12

    # Calculate the angular acceleration
    angular_acceleration = torque / moment_of_inertia

    # Calculate the change in rotational velocity
    change_in_rotational_velocity = angular_acceleration * time_step

    return change_in_rotational_velocity

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-20, 20])
ax.set_ylim([-20, 20])
ax.set_zlim([0, 20])

fig2 = plt.figure()
x_angle_plot = fig2.add_subplot(111)
x_angle_plot.set_xlim([0, 100])
x_angle_plot.set_ylim([-90, 90])

fig3 = plt.figure()
y_angle_plot = fig3.add_subplot(111)
y_angle_plot.set_xlim([0, 100])
y_angle_plot.set_ylim([-90, 90])

figure, axis = plt.subplots(2, 2)






# Inputs
launch_angle_x_degrees = 20  # in degrees
launch_angle_y_degrees = -35  # in degrees
gravity_acceleration = -9.81  # in meters per second squared

x_velocity = 0
y_velocity = 0
z_velocity = 0

x = 0
y = 0
z = 0

time = 0
step = 0.01

rotational_velocity = [0, 0]
thrust_vectors = []
vector_angles = [0, 0]
acc_vector = []

max_thrust = 20
throttle_percentage = 100

mass = 1.5
cg_to_gimbal_distance = 0.6
length = 1.5

burn_time = 50

x_axis_P = 5
x_axis_I = 0.02
x_axis_D = 0.7

x_integral = 0

y_axis_P = 5
y_axis_I = 0.02
y_axis_D = 0.7

y_integral = 0

throttle_P = 290
throttle_I = 15
throttle_D = 12

counter = 0
counter2 = 0
throttle_integral = 0
x_angle_setpoint = 0
y_angle_setpoint = 0

while z >= -1:
    x_angle_setpoint = x_velocity * 6
    y_angle_setpoint = y_velocity * -6

    counter += 1
    counter2 += 1
    x_integral += launch_angle_x_degrees * step
    y_integral += launch_angle_y_degrees * step
    throttle_integral += (ascent_rate(time) - z_velocity) * step
    vector_angles[0] = PID(x_axis_P, x_axis_I, x_axis_D, launch_angle_x_degrees - x_angle_setpoint, rotational_velocity[0], x_integral)
    vector_angles[1] = -PID(y_axis_P, y_axis_I, y_axis_D, launch_angle_y_degrees - y_angle_setpoint, rotational_velocity[1], y_integral)

    if vector_angles[0] > 30:
        vector_angles[0] = 30
    elif vector_angles[0] < -30:
        vector_angles[0] = -30
    if vector_angles[1] > 30:
        vector_angles[1] = 30
    elif vector_angles[1] < -30:
        vector_angles[1] = -30

    thrust_vectors = force_vectors(vector_angles, max_thrust * throttle_percentage / 100)
    rotational_velocity[0] += math.degrees(change_in_rotational_velocity(thrust_vectors[0], cg_to_gimbal_distance, mass, length, step))
    rotational_velocity[1] += math.degrees(change_in_rotational_velocity(thrust_vectors[1], cg_to_gimbal_distance, mass, length, step))

    launch_angle_x_degrees += rotational_velocity[0] * step
    launch_angle_y_degrees += rotational_velocity[1] * step

    burn_time -= throttle_percentage / 100 * step
    if burn_time > 0:
        acc_vector = force_vectors([launch_angle_x_degrees, launch_angle_y_degrees], thrust_vectors[2])
        #print(acc_vector)
        # Plot the current position
        if counter == 30:
            ax.plot([x], [y], [z], 'bo')
            counter = 0
    else:
        acc_vector = [0, 0, 0]
        # Plot the current position
        if counter == 30:
            ax.plot([x], [y], [z], 'ro')
            counter = 0

    if counter2 == 10:
        x_angle_plot.plot([time], [launch_angle_x_degrees], 'bo')
        y_angle_plot.plot([time], [launch_angle_y_degrees], 'bo')

        axis[0,0].plot([time], [throttle_percentage], 'bo')
        axis[0,1].plot([time], [vector_angles[0]], 'bo')
        axis[1,0].plot([time], [vector_angles[1]], 'bo')
        axis[1,1].plot([time], z_velocity, 'bo')
        counter2 = 0


    throttle_percentage = PID(throttle_P, throttle_I, throttle_D, ascent_rate(time) - z_velocity, acc_vector[2] / mass * step, throttle_integral)

    if throttle_percentage > 100:
        throttle_percentage = 100
    elif throttle_percentage < 0:
        throttle_percentage = 0

    x_velocity += acc_vector[0] / mass * step
    y_velocity += acc_vector[1] / mass * step
    z_velocity += acc_vector[2] / mass * step

    #print(x_velocity, y_velocity, z_velocity)

    # Simulate projectile motion
    x += x_velocity * step
    y += y_velocity * step
    z += z_velocity * step


    z_velocity += gravity_acceleration * step


    time += step

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Projectile Motion Trajectory')

x_angle_plot.set_xlabel('Time')
x_angle_plot.set_ylabel('X Angle')

y_angle_plot.set_xlabel('Time')
y_angle_plot.set_ylabel('Y Angle')

axis[0,0].set_xlabel('Time')
axis[0,0].set_ylabel('Throttle')

axis[0,1].set_xlabel('Time')
axis[0,1].set_ylabel('X Vector Angle')

axis[1,0].set_xlabel('Time')
axis[1,0].set_ylabel('Y Vector Angle')

axis[1,1].set_xlabel('Time')
axis[1,1].set_ylabel('Z Velocity')

plt.show()