from math import floor
from multiprocessing import Pool
from time import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

DEBUG_SHOT = False
DEBUG_SHOT_DISTANCE = 1
DEBUG_SHOT_ROBOT_RADIAL_VELOCITY = 0
DEBUG_RANGE = True
DEBUG_SHOT_DISTANCE_RANGE = 6
DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE = 5

MAGNUS_EFFECT_ENABLED = False

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

# All length quantities are in meters

fuel_mass = 0.2150028  # kg - note, this is the average weight according to the range in the game manual
fuel_radius = (15 / 100) / 2

hubz = 72 * 2.54 / 100
# Add some clearance to the top of the hub edge (15.5 + clearance)
hub_edgez = hubz + (15.5 + 5) * 2.54 / 100 + fuel_radius
hub_edgex_offset = -24 * 2.54 / 100

dt = 0.005
t_final = 2
t = np.linspace(0, t_final, round(t_final / dt))

g = 9.81
# Drag force and magnus effect coefficients. See:
# - https://en.wikipedia.org/wiki/Drag_equation
# - https://www.physics.usyd.edu.au/~cross/TRAJECTORIES/42.%20Ball%20Trajectories.pdf
# - https://www.chiefdelphi.com/t/paper-ballistic-trajectory-with-air-friction-drag-and-magnus/123764
ρ = 1.2041  # air, kg/m³, https://en.wikipedia.org/wiki/Density_of_air#Dry_air
A = np.pi * fuel_radius ** 2
C_D = 0.47  # https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg

def deg_to_rad(deg):
    return deg / 180.0 * np.pi

def rad_to_deg(rad):
    return rad / np.pi * 180.0

def polar_velocity_to_components(vel, pitch, yaw=0.0):
    vz = vel * np.sin(pitch)
    vxy_hypot = vel * np.cos(pitch)
    vx = vxy_hypot * np.cos(yaw)
    vy = vxy_hypot * np.sin(yaw)

    return vx, vy, vz

def cross(a, b):
    # https://en.wikipedia.org/wiki/Cross_product#Coordinate_notation
    return np.array([
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ])

def norm(v):
    return np.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

def normalize(v):
    return v / norm(v)

def calculate_trajectory_kinematics(vel, hood_angle, robot_radial_velocity):
    vx, vy, vz = polar_velocity_to_components(vel, hood_angle)
    vx += robot_radial_velocity

    res = (
        vx * t,
        vy * t,
        vz * t + (1 / 2) * -g * t ** 2
    )

    # Only return trajectory until we go into the hub, if possible
    past_hub_on_upwards_arc = False
    for i in range(len(t)):
        if res[2][i] > hubz:
            past_hub_on_upwards_arc = True
        elif res[2][i] < hubz and past_hub_on_upwards_arc:
            i_end = i + 1
            return res[0][:i_end], res[1][:i_end], res[2][:i_end]

    return res

def calculate_trajectory_iterative(vel, hood_angle, robot_radial_velocity):
    vx, vy, vz = polar_velocity_to_components(vel, hood_angle)
    vx += robot_radial_velocity

    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = np.zeros_like(t)
    past_hub_on_upwards_arc = False

    for i in range(len(t)):
        # Get last position, velocity, acceleration
        if i > 0:
            lx = x[i - 1]
            ly = y[i - 1]
            lz = z[i - 1]
        else:
            lx = ly = lz = 0

        lvx = vx
        lvy = vy
        lvz = vz

        lv = np.array([lvx, lvy, lvz])
        lv_unit = normalize(lv)
        lv_mag = np.sqrt(lvx ** 2 + lvy ** 2 + lvz ** 2)

        if i > 0:
            lax = ax
            lay = ay
            laz = az
        else:
            lax = lay = laz = 0

        # Calculate forces
        f = np.array([0.0, 0.0, 0.0])

        ## Weight
        f[2] += fuel_mass * -g

        ## Drag. See: https://en.wikipedia.org/wiki/Drag_equation
        ### Get current velocity as unit vector and magnitude
        fd = (1 / 2) * ρ * lv_mag ** 2 * C_D * A
        ### Drag is opposite of current velocity unit vector
        fd = -fd * lv_unit
        f += fd

        ## Magnus effect. See:
        ## - https://www.physics.usyd.edu.au/~cross/TRAJECTORIES/42.%20Ball%20Trajectories.pdf
        ## - https://www.chiefdelphi.com/t/paper-ballistic-trajectory-with-air-friction-drag-and-magnus/123764
        ### TODO: calculate angular velocity based on initial velocity
        ### TODO: angular velocity drag
        ω = 10 * normalize(np.array([-lv[1], lv[0], 0]))  # Rotation axis is motion direction rotated by 90° CCW
        C_L = 1 / (2 + (lv_mag / (lv_mag + 1)))
        if C_L > C_D:
            C_L = C_D
        fm = (1 / 2) * ρ * A * fuel_radius * C_L * lv_mag * cross(ω, lv)
        if MAGNUS_EFFECT_ENABLED:
            f += fm
            # print(fm)
            # print(normalize(cross(lv_unit, axis_of_rotation)))

        # Calculate acceleration
        ax = f[0] / fuel_mass
        ay = f[1] / fuel_mass
        az = f[2] / fuel_mass

        # Integrate acceleration
        vx += (lax + ax) / 2 * dt
        vy += (lay + ay) / 2 * dt
        vz += (laz + az) / 2 * dt

        # Integrate velocity
        x[i] = lx + (lvx + vx) / 2 * dt
        y[i] = ly + (lvy + vy) / 2 * dt
        z[i] = lz + (lvz + vz) / 2 * dt

        # Only return trajectory until we go into the hub, if possible
        if z[i] > hubz:
            past_hub_on_upwards_arc = True
        elif z[i] < hubz and past_hub_on_upwards_arc:
            i_end = i + 1
            return x[:i_end], y[:i_end], z[:i_end]

    return x, y, z

def calculate_shooting_params_kinematics(distance, robot_radial_vel):
    # https://www.desmos.com/calculator/9npcb4woqc
    v0 = 0.0742955 * distance ** 2 + 0.185739 * distance + 6.16695
    vr = robot_radial_vel

    # print(f"v0 = {v0}, vr = {vr}")

    # First compute stationary shooting velocity
    discriminant = v0 ** 4 - g * (g * distance ** 2 + 2 * hubz * v0 ** 2)
    if discriminant < 0:
        print("\tDiscriminant is negative")
        exit(1)
    phi_1 = np.atan((v0 ** 2 + np.sqrt(discriminant)) / (g * distance))
    phi_2 = np.atan((v0 ** 2 - np.sqrt(discriminant)) / (g * distance))

    def is_valid_phi(phi):
        return deg_to_rad(15) < phi < deg_to_rad(90)

    if is_valid_phi(phi_1) and (phi_1 > phi_2 or not is_valid_phi(phi_2)):
        hood_angle = phi_1
    elif is_valid_phi(phi_2):
        hood_angle = phi_2
    else:
        print("\tNo phi found")
        exit(1)

    # print(f"\tphi_1 = {rad_to_deg(phi_1)}, phi_2 = {rad_to_deg(phi_2)}, hood_angle = {rad_to_deg(hood_angle)}")

    vx = v0 * np.cos(hood_angle)
    vy = 0
    vz = v0 * np.sin(hood_angle)

    # Now subtract robot velocity from stationary shooting velocity
    vx -= vr

    # Now calculate hood_angle and shooting magnitude from 3d shooting vector
    v = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    hood_angle = np.asin(vz / v)
    # print(f"\tafter compensate: v = {v}, hood_angle = {rad_to_deg(hood_angle)}")

    return v, hood_angle

def optimize_shot(distance, robot_radial_vel):
    hubx = distance
    hub_edgex = hubx + hub_edgex_offset
    ax.scatter(hubx, 0, hubz, c="red")
    ax.scatter(hub_edgex, 0, hub_edgez, c="red")

    v_initial, hood_angle_initial = calculate_shooting_params_kinematics(distance, robot_radial_vel)

    def cost_fun(x):
        x, y, z = calculate_trajectory_iterative(x[0], x[1], robot_radial_vel)

        # Find X distance to hub
        x_dist = abs(x[-1] - hubx)
        if x_dist < fuel_radius:
            x_dist = 0

        # Find max Z
        # If we are too close, who cares
        if hubx >= 2:
            max_z = -1
            for some_z in z:
                if some_z > max_z:
                    max_z = some_z
            # Target a certain max z based on distance
            max_z = abs(max_z - (1 / 2) * hubx)
            # Reduce significance
            max_z /= 2
        else:
            max_z = 0

        # Find i where x is closest to edge
        closest_i = len(x) - 1
        for i in range(len(x)):
            if abs(x[i] - hub_edgex) < abs(x[closest_i] - hub_edgex):
                closest_i = i
        # Get Z distance when X is at the edge
        if z[closest_i] < hub_edgez:
            # If we are below the edge, bad
            z_dist = abs(z[closest_i] - hub_edgez)
            # Increase significance
            z_dist *= 2
        else:
            z_dist = 0

        return x_dist + max_z + z_dist

    res = minimize(
        cost_fun,
        np.array([v_initial, hood_angle_initial]),
        method="Nelder-Mead",
        options={"maxiter": 800}
    )

    if not res.success or cost_fun(res.x) > 0.1:
        print(f"Optimization failed.")
        print(f"\tdistance = {distance}, robot_radial_vel = {robot_radial_vel}")
        print(f"\tres = {res}")
        return None, None

    v_final, hood_angle_final = res.x

    if DEBUG_SHOT:
        print(res)
        # ax.plot(*calculate_trajectory_kinematics(v_initial, hood_angle_initial, robot_radial_vel), label="Simple Kinematics (Initial)")
        # ax.plot(*calculate_trajectory_kinematics(v_final, hood_angle_final, robot_radial_vel), label="Simple Kinematics (Final)")
        # ax.plot(*calculate_trajectory_iterative(v_initial, hood_angle_initial, robot_radial_vel), label="Iterative Simulation (Initial)")
        ax.plot(
            *calculate_trajectory_iterative(v_final, hood_angle_final, robot_radial_vel),
            label="Iterative Simulation (Final)" if not DEBUG_RANGE else None
        )

    return v_final, hood_angle_final

if DEBUG_SHOT:
    if DEBUG_RANGE:
        optimize_shot(DEBUG_SHOT_DISTANCE, DEBUG_SHOT_ROBOT_RADIAL_VELOCITY)
        for i in range(DEBUG_SHOT_DISTANCE_RANGE + 1):
            for j in range(DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE):
                optimize_shot(DEBUG_SHOT_DISTANCE + i, j - (DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE / 2))
    else:
        optimize_shot(DEBUG_SHOT_DISTANCE, DEBUG_SHOT_ROBOT_RADIAL_VELOCITY)

    ax.set(ylim=[-2, 2], zlim=[0, hubz + 2], xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    ax.legend()

    plt.show()
else:
    def shot_compute_worker(x):
        worker_index, worker_distance_velocity_pairs = x

        shots = []
        counter = 0

        print(f"[{worker_index}] Starting")
        start_time_worker = time()

        for distance, velocity in worker_distance_velocity_pairs:
            counter += 1
            progress_count = f"{counter}/{len(worker_distance_velocity_pairs)}"
            progress_percent = round(float(counter) / float(len(worker_distance_velocity_pairs)) * 100)

            v, hood_angle = optimize_shot(distance, velocity)
            if v is None and hood_angle is None:
                print(
                    f"[{worker_index}] {progress_count} FAILED (distance = {distance}, velocity = {velocity})")
                continue
            shots.append(((distance, velocity), {v, hood_angle}))
            print(f"[{worker_index}] {progress_count}\t{progress_percent}%")

        end_time_worker = time()
        print(f"[{worker_index}] Done, took {end_time_worker - start_time_worker} seconds")

        return shots

        # counter = 0

    if __name__ == "__main__":
        distance_velocity_pairs = []

        start_dist = 0.5
        stop_dist = 7
        max_vel = 4.5
        for distance in np.linspace(start_dist, stop_dist, 70):
            max_vel_away_from_hub = -max_vel
            max_vel_towards_hub = min(max_vel, 1 + 2 * distance)
            for velocity in np.linspace(max_vel_away_from_hub, max_vel_towards_hub, 20):
                distance_velocity_pairs.append((distance, velocity))

        all_shots = []
        workers = 8
        min_per_worker = floor(len(distance_velocity_pairs) / workers)

        print(
            f"Computing {len(distance_velocity_pairs)} distance velocity pairs,"
            f"with a minimum of {min_per_worker} per worker"
        )

        def get_distance_velocity_pairs_for_worker(worker_index):
            start_index = worker_index * min_per_worker
            if worker_index == workers - 1:
                end_index = len(distance_velocity_pairs)
            else:
                end_index = start_index + min_per_worker
            print(f"Worker {worker_index} will do indices {start_index} to {end_index - 1}")
            return worker_index, distance_velocity_pairs[start_index:end_index]

        start_time = time()

        with Pool(workers) as p:
            args = map(get_distance_velocity_pairs_for_worker, range(workers))
            for computed_shots in p.map(shot_compute_worker, args):
                all_shots += computed_shots

        end_time = time()
        print(f"Took {end_time - start_time} seconds")
