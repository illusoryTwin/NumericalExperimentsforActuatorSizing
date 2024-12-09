import time
import numpy as np
import mujoco
import mujoco.viewer
import json

# Load the robot model
model_name = "2_robot_model"
model_path = f"../model/{model_name}.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

waypoints = [np.deg2rad([80]*6),
            np.deg2rad([10, 10, 10, 10, 10, 10]),
            np.deg2rad([15]*6),
            np.deg2rad([20]*6)]

# Parameters
speed = 0.1  # Speed in radians per second
time_step = model.opt.timestep  # Simulation timestep
torques = []
current_position = np.array(data.qpos)  # Initial joint positions

# Interpolation function
def interpolate_waypoints(start, end, step_size):
    direction = end - start
    distance = np.linalg.norm(direction)
    steps = int(distance / step_size)
    return np.linspace(start, end, steps, axis=0)

# Initialize viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    iteration = 0

    while viewer.is_running() and iteration < len(waypoints) - 1:
        start_waypoint = np.deg2rad(np.array(waypoints[iteration]))
        end_waypoint = np.deg2rad(np.array(waypoints[iteration + 1]))

        # Interpolate between waypoints
        step_size = speed * time_step  # Step size based on speed and timestep
        interpolated_points = interpolate_waypoints(start_waypoint, end_waypoint, step_size)
        print("interpolated_points", interpolated_points)
        for point in interpolated_points:
            step_start = time.time()
            mujoco.mj_forward(model, data)

            data.qpos = point
            data.qacc = np.zeros_like(data.qacc)  # Zero acceleration

            # Compute torques using inverse dynamics
            mujoco.mj_inverse(model, data)
            print("data.qfrc_inverse", data.qfrc_inverse)
            torques.append(data.qfrc_inverse)

            # Step the simulation
            viewer.sync()

            # Control the simulation step time
            time_until_next_step = time_step - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        iteration += 1  # Move to the next waypoint
        if time.time() - start_time > 300:
            print("Simulation timed out.")
            break
