import matplotlib.pyplot as plt
import numpy as np
from math import floor
from multiprocessing import Pool
from os.path import realpath, dirname
from scipy.linalg import inv
from scipy.optimize import curve_fit
from time import time

DEBUG_SHOT = False
DEBUG_SHOT_DISTANCE = 0.7
DEBUG_SHOT_ROBOT_RADIAL_VELOCITY = 0

DEBUG_DISTANCE_RANGE = True
DEBUG_SHOT_DISTANCE_RANGE = 6

DEBUG_VELOCITY_RANGE = False
DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE = 6

MAGNUS_EFFECT_ENABLED = False

if DEBUG_SHOT:
    fig, ax = plt.subplots()  # subplot_kw=dict(projection="3d"))

def inches_to_meters(inches):
    return inches * 2.54 / 100

rad_to_deg = np.degrees
deg_to_rad = np.radians

# All length quantities are in meters

fuel_mass = 0.2150028  # kg - note, this is the average weight according to the range in the game manual
# todo: maybe add some random mass difference to account for error?
fuel_radius = (15 / 100) / 2

# KEEP SYNCED WITH DriveConstants.java
wheel_radius = inches_to_meters(1.945)
bottom_of_frame_rails_to_center_of_wheels = inches_to_meters(-0.247776)

# KEEP SYNCED WITH ShootingKinematics.java
bottom_of_frame_rails_to_shooter_height = inches_to_meters(12.861380)
shooter_radius_to_center_of_ball_exit = inches_to_meters(4.602756)

z_initial_base = bottom_of_frame_rails_to_center_of_wheels + wheel_radius + bottom_of_frame_rails_to_shooter_height

# From horizontal
min_angle_allowed = deg_to_rad(50.0)
max_angle_allowed = deg_to_rad(75.0)

hub_base_z = inches_to_meters(56.5)
hub_base_x = inches_to_meters(11.914689)  # Half of inner radius of base

hub_edge_z = inches_to_meters(72.0)
hub_edge_x = inches_to_meters(20.863618)  # Half of the inner radius of the edge

hub_clearance_z = inches_to_meters(5) + fuel_radius
hub_edge_z_with_clearance = hub_edge_z + hub_clearance_z

def hub_z(x):
    # In base
    if abs(x) < hub_base_x:
        return hub_base_z
    # Outside of edge
    if abs(x) > hub_edge_x:
        return hub_edge_z
    # Interpolate
    return hub_base_z + (abs(x) - hub_base_x) * (hub_edge_z - hub_base_z) / (hub_edge_x - hub_base_x)

dist_to_hub_x_data = np.array([
    [0.7, -0.4],
    [1.2, 0.5],
    [1.7, 0.2],
    [3.5, 0.2],
    [6.5, 0.37],
])
get_wanted_hub_x = lambda d: np.interp(d, dist_to_hub_x_data[:, 0], dist_to_hub_x_data[:, 1])
# Hub X debug
# ax.scatter(dist_to_hub_x_data[:, 0], dist_to_hub_x_data[:, 1])
# distance = np.linspace(0, 10, 200)
# ax.plot(distance, [get_wanted_hub_x(d) for d in distance])
# plt.show()

dist_to_entry_angle_data = np.array([
    [0.7, deg_to_rad(-88)],
    [1.2, deg_to_rad(-63)],
    [2.2, deg_to_rad(-63.5)],
    [4.2, deg_to_rad(-53)],
    [6.2, deg_to_rad(-48.4)],
    [8, deg_to_rad(-47)],
])
get_wanted_entry_angle = lambda d: np.interp(d, dist_to_entry_angle_data[:, 0], dist_to_entry_angle_data[:, 1])
# Entry angle debug
# ax.scatter(dist_to_entry_angle_data[:, 0], dist_to_entry_angle_data[:, 1])
# distance = np.linspace(0, 10, 200)
# ax.plot(distance, [get_wanted_entry_angle(d) for d in distance])
# plt.show()

if DEBUG_SHOT:
    ax.scatter(-hub_edge_x, hub_edge_z_with_clearance, c="red")
    ax.scatter(-hub_edge_x - fuel_radius, hub_edge_z, c="red")
    ax.plot(
        [
            -hub_edge_x,
            -hub_base_x,
            hub_base_x,
            hub_edge_x,
        ],
        [
            hub_edge_z,
            hub_base_z,
            hub_base_z,
            hub_edge_z,
        ],
        c="green"
    )

    # x = np.linspace(-0.6, 0.6, 50)
    # ax.plot(x, [hub_z(x) + hub_clearance_z for x in x])

dt = 0.005
t_final = 3
t = np.linspace(0, t_final, round(t_final / dt))

# Use larger than real gravity value to approximate drag and stuff
g = 11  # 9.81
# Drag force and magnus effect coefficients. See:
# - https://en.wikipedia.org/wiki/Drag_equation
# - https://www.physics.usyd.edu.au/~cross/TRAJECTORIES/42.%20Ball%20Trajectories.pdf
# - https://www.chiefdelphi.com/t/paper-ballistic-trajectory-with-air-friction-drag-and-magnus/123764
ρ = 1.2041  # air, kg/m³, https://en.wikipedia.org/wiki/Density_of_air#Dry_air
A = np.pi * fuel_radius ** 2
C_D = 0.47  # https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg

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

def calculate_trajectory_kinematics(vel, angle, robot_radial_velocity, x0):
    vx, vy, vz = polar_velocity_to_components(vel, angle)
    vx += robot_radial_velocity

    x = x0 + vx * t
    y = vy * t
    z = vz * t + (1 / 2) * -g * t ** 2

    # Only return trajectory until we go into the hub, if possible
    past_hub_on_upwards_arc = False
    for i in range(len(t)):
        if z[i] > hub_z(x[i]):
            past_hub_on_upwards_arc = True
        elif z[i] < hub_z(x[i]) and past_hub_on_upwards_arc:
            i_end = i + 1
            return x[:i_end], y[:i_end], z[:i_end]

    return x, y, z

def calculate_trajectory_iterative(vel, angle, robot_radial_velocity, x0):
    vx, vy, vz = polar_velocity_to_components(vel, angle)
    vx += robot_radial_velocity

    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = np.zeros_like(t)
    past_hub_on_upwards_arc = False

    for i in range(len(t)):
        if i > 0:
            # Get last position, velocity, acceleration
            lx = x[i - 1]
            ly = y[i - 1]
            lz = z[i - 1]
        else:
            # Get initial position
            # ShootingKinematics.java measures distance including the X offset due to angle
            # so we don't need to include the X offset here
            lx = x0
            # lx = (shooter_radius_to_center_of_ball_exit +
            #      np.cos(np.pi / 2.0 - angle) * -shooter_radius_to_center_of_ball_exit)
            ly = 0
            lz = z_initial_base + np.sin(np.pi / 2.0 - angle) * shooter_radius_to_center_of_ball_exit

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
        if MAGNUS_EFFECT_ENABLED:
            ω = 10 * normalize(np.array([-lv[1], lv[0], 0]))  # Rotation axis is motion direction rotated by 90° CCW
            C_L = 1 / (2 + (lv_mag / (lv_mag + 1)))
            if C_L > C_D:
                C_L = C_D
            fm = (1 / 2) * ρ * A * fuel_radius * C_L * lv_mag * cross(ω, lv)
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

        # We don't currently graph y. Make sure it is always zero
        assert y[i] == 0.0

        # Find angle of velocity
        vel_angle = np.atan2(vz, vx)

        # Only return trajectory until we go into the hub, if possible
        if z[i] > hub_z(x[i]):
            past_hub_on_upwards_arc = True
        elif z[i] < hub_z(x[i]) and past_hub_on_upwards_arc:
            i_end = i + 1
            return x[:i_end], y[:i_end], z[:i_end], vel_angle

    return x, y, z, vel_angle

def calculate_shooting_params_kinematics(distance, robot_radial_vel):
    # https://www.desmos.com/calculator/9npcb4woqc
    v0 = 0.0742955 * distance ** 2 + 0.185739 * distance + 6.16695 + 2
    vr = robot_radial_vel

    # print(f"v0 = {v0}, vr = {vr}")

    # First compute stationary shooting velocity
    discriminant = v0 ** 4 - g * (g * distance ** 2 + 2 * (hub_base_z + hub_clearance_z) * v0 ** 2)
    if discriminant < 0:
        print("\tDiscriminant is negative")
        exit(1)
    phi_1 = np.atan((v0 ** 2 + np.sqrt(discriminant)) / (g * distance))
    phi_2 = np.atan((v0 ** 2 - np.sqrt(discriminant)) / (g * distance))

    def is_valid_phi(phi):
        return deg_to_rad(15) < phi < deg_to_rad(90)

    if is_valid_phi(phi_1) and (phi_1 > phi_2 or not is_valid_phi(phi_2)):
        angle = phi_1
    elif is_valid_phi(phi_2):
        angle = phi_2
    else:
        print("\tNo phi found")
        exit(1)

    # print(f"\tphi_1 = {rad_to_deg(phi_1)}, phi_2 = {rad_to_deg(phi_2)}, angle = {rad_to_deg(angle)}")

    vx = v0 * np.cos(angle)
    vy = 0
    vz = v0 * np.sin(angle)

    # Now subtract robot velocity from stationary shooting velocity
    vx -= vr

    # Now calculate angle and shooting magnitude from 3d shooting vector
    v = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    angle = np.asin(vz / v)
    # print(f"\tafter compensate: v = {v}, angle = {rad_to_deg(angle)}")

    return v, angle

def optimize_shot(distance, robot_radial_vel):
    x0 = -distance

    wanted_x = get_wanted_hub_x(distance)
    wanted_entry_angle = get_wanted_entry_angle(distance)

    x_tolerance = inches_to_meters(0.5)
    entry_angle_tolerance = deg_to_rad(0.5)

    v_initial, shot_angle_initial = calculate_shooting_params_kinematics(distance, robot_radial_vel)

    shots_simmed = 0

    Δv = 0.1
    Δshot_angle = 0.1
    α = 0.1

    i = 0
    max_iterations = 70

    v = v_initial
    shot_angle = shot_angle_initial

    def solve():
        nonlocal i, v, shot_angle, shots_simmed
        while i < max_iterations:
            i += 1

            # Newton's method in two dimensions
            # Compute initial guess
            x, y, z, entry_angle_1 = calculate_trajectory_iterative(v, shot_angle, robot_radial_vel, x0)
            steps = len(x)
            x_1 = x[-1]
            shots_simmed += 1
            if DEBUG_SHOT and not DEBUG_DISTANCE_RANGE and not DEBUG_VELOCITY_RANGE and i % (max_iterations / 10) == 0:
                ax.plot(x, z, linestyle="dotted", c=(1 - i / max_iterations, i / max_iterations, 0))

            # If guess is within tolerance, exit early
            # print(i, abs(x_1 - wanted_x) <= x_tolerance, abs(entry_angle_1 - wanted_entry_angle) <= entry_angle_tolerance)
            if (abs(x_1 - wanted_x) <= x_tolerance and abs(
                    entry_angle_1 - wanted_entry_angle) <= entry_angle_tolerance):
                break

            # Compute guess with velocity increment
            x, y, z, entry_angle_2 = calculate_trajectory_iterative(v + Δv, shot_angle, robot_radial_vel, x0)
            x_2 = x[-1]
            shots_simmed += 1

            # Compute guess with angle increment
            x, y, z, entry_angle_3 = calculate_trajectory_iterative(v, shot_angle + Δshot_angle, robot_radial_vel, x0)
            x_3 = x[-1]
            shots_simmed += 1

            # Compute matrix
            A_11 = (x_2 - x_1) / Δv
            A_12 = (x_3 - x_1) / Δshot_angle
            A_21 = (entry_angle_2 - entry_angle_1) / Δv
            A_22 = (entry_angle_3 - entry_angle_1) / Δshot_angle
            A = np.array([[A_11, A_12], [A_21, A_22]])

            # Compute vector difference
            d = np.array([wanted_x - x_1, wanted_entry_angle - entry_angle_1])

            # Compute matrix inverse
            A_inv = inv(A)

            # Compute full increment
            Δ = np.matmul(A_inv, d)

            # Update guess
            v += α * Δ[0]
            shot_angle += α * Δ[1]
        else:
            # If we don't break (that's what the else clause checks for), check the guess once more.
            # If it doesn't satisfy tolerances, solution could not be found
            x, y, z, entry_angle = calculate_trajectory_iterative(v, shot_angle, robot_radial_vel, x0)
            steps = len(x)
            x = x[-1]
            shots_simmed += 1

            if (abs(x - wanted_x) > x_tolerance or
                    abs(entry_angle - wanted_entry_angle) > entry_angle_tolerance):
                return None

        if v < 0 or v > 30:
            # Sanity check because sometimes the solver goes crazy
            return None

        return steps

    steps = solve()
    if steps is None:
        x_tolerance *= 2
        entry_angle_tolerance *= 2

        Δv *= 0.5
        Δshot_angle *= 0.1
        α *= 0.5

        max_iterations += 300

        v = v_initial
        shot_angle = shot_angle_initial

        steps = solve()
        if steps is None:
            print(f"Could not find solution after {i} iterations.")
            print(f"\tdistance = {distance}, robot_radial_vel = {robot_radial_vel}")
            print(f"\tv = {v}, shot_angle = {shot_angle}")
            return None, None, None, None, None

    # We don't actually want to fail the shot even if it is invalid.
    # Without the shot in the data set, the robot could try to execute a shot
    # that is guaranteed to miss. With the shot in the data set, the robot will
    # know that it cannot shoot if it wants to make the shot. However, it's still useful to know
    # when a solution is invalid so that entry angles can be tuned.
    if not (max_angle_allowed > shot_angle > min_angle_allowed):
        print(f"Found invalid solution")
        print(f"\tdistance = {distance}, robot_radial_vel = {robot_radial_vel}")
        print(f"\tv = {v}, shot_angle = {shot_angle}")

    tof = t[steps - 1]

    if DEBUG_SHOT:
        print(f"Found valid solution after {i} iterations")
        print(f"\tdistance = {distance}, robot_radial_vel = {robot_radial_vel}")
        print(f"\tv = {v}, shot_angle = {shot_angle}")
        print(f"\ttof: {tof}")
        # ax.plot(*calculate_trajectory_kinematics(v_initial, angle_initial, robot_radial_vel), label="Simple Kinematics (Initial)")
        # ax.plot(*calculate_trajectory_kinematics(v_final, angle_final, robot_radial_vel), label="Simple Kinematics (Final)")
        # ax.plot(*calculate_trajectory_iterative(v_initial, angle_initial, robot_radial_vel), label="Iterative Simulation (Initial)")
        x, y, z, _ = calculate_trajectory_iterative(v, shot_angle, robot_radial_vel, x0)
        ax.plot(
            x, z,
            # label="Iterative Simulation (Final)" if not DEBUG_RANGE else None
        )

    return v, shot_angle, tof, shots_simmed, i

if DEBUG_SHOT:
    optimize_shot(DEBUG_SHOT_DISTANCE, DEBUG_SHOT_ROBOT_RADIAL_VELOCITY)
    if DEBUG_DISTANCE_RANGE and DEBUG_VELOCITY_RANGE:
        for i in range((DEBUG_SHOT_DISTANCE_RANGE + 1) * 2):
            for j in range(DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE):
                optimize_shot(DEBUG_SHOT_DISTANCE + i / 2, j - (DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE / 2))
    elif DEBUG_DISTANCE_RANGE:
        for i in range((DEBUG_SHOT_DISTANCE_RANGE + 1) * 2):
            optimize_shot(DEBUG_SHOT_DISTANCE + i / 2, DEBUG_SHOT_ROBOT_RADIAL_VELOCITY)
    elif DEBUG_VELOCITY_RANGE:
        for j in range(DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE):
            optimize_shot(DEBUG_SHOT_DISTANCE, j - (DEBUG_SHOT_ROBOT_RADIAL_VELOCITY_RANGE / 2))

    # ax.set(ylim=[-2, 2], zlim=[0, hubz + 2], xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    # ax.set_ylim([0, abs(ax.get_xlim()[0]) + abs(ax.get_xlim()[1])])
    ax.legend()

    plt.show()
else:
    def shot_compute_worker(x):
        worker_index, worker_distance_velocity_pairs = x

        shots = []
        counter = 0
        all_shots_simmed = 0

        print(f"[{worker_index}] Starting")
        start_time_worker = time()

        for distance, velocity in worker_distance_velocity_pairs:
            counter += 1
            progress_count = f"{counter}/{len(worker_distance_velocity_pairs)}"
            progress_percent = round(float(counter) / float(len(worker_distance_velocity_pairs)) * 100)

            v, angle, tof, shots_simmed, iterations = optimize_shot(distance, velocity)
            if v is None and angle is None and tof is None:
                print(
                    f"[{worker_index}] {progress_count} FAILED (distance = {distance}, velocity = {velocity})")
                continue
            all_shots_simmed += shots_simmed
            # the two tuples need to be the same length for numpy to be happy, so add the extra 0
            shots.append(((distance, velocity, 0), (v, angle, tof)))
            print(f"[{worker_index}] {progress_count}\t{progress_percent}% ({shots_simmed}, {iterations})")

        end_time_worker = time()
        print(
            f"[{worker_index}] Done, took {end_time_worker - start_time_worker} seconds and {all_shots_simmed} simulations"
        )

        return shots, all_shots_simmed

        # counter = 0

    if __name__ == "__main__":
        distance_velocity_pairs = []

        start_dist = 0.7
        stop_dist = 7
        max_vel = 4.5
        for distance in np.linspace(start_dist, stop_dist, 50):
            for velocity in [-max_vel, -3.0, -2.25, -1.5, -0.75, 0.0, 0.75, 1.5, 2.25, 3.0, max_vel]:
                distance_velocity_pairs.append((distance, velocity))

        all_shots = []
        all_shots_simmed = 0
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
            for (computed_shots, shots_simmed) in p.map(shot_compute_worker, args):
                all_shots += computed_shots
                all_shots_simmed += shots_simmed

        end_time = time()
        print(f"Took {end_time - start_time} seconds")
        print(f"Of {len(distance_velocity_pairs)} shots, {len(all_shots)} succeeded.")
        print(f"{all_shots_simmed} simulations run.")

        all_shots = np.array(all_shots)

        X = (all_shots[:, 0, 0], all_shots[:, 0, 1])
        y_vel = all_shots[:, 1, 0]
        y_angle = all_shots[:, 1, 1]
        y_tof = all_shots[:, 1, 2]

        print("X", X)
        print("y_vel", y_vel)
        print("y_angle", y_angle)
        print("y_tof", y_tof)

        def f(X, i0, i1, i2, i3, i4, i5):  # , i6, i7, i8, i9):
            x = X[0]
            y = X[1]
            return (
                    i0 +
                    + i1 * x
                    + i2 * y
                    + i3 * x * y
                    + i4 * x * x
                    + i5 * y * y
                # + i6 * x * x * y * y
                # + i7 * x * x * x
                # + i8 * y * y * y
                # + i9 * x * x * x * y * y * y
            )

        def make_regression(X, y):
            coeff, cov = curve_fit(f, X, y)
            equation = (
                f"{coeff[0]}"
                f" + {coeff[1]} * x"
                f" + {coeff[2]} * y"
                f" + {coeff[3]} * x * y"
                f" + {coeff[4]} * x * x"
                f" + {coeff[5]} * y * y"
                # f" + {coeff[6]} * x * x * y * y"
                # f" + {coeff[7]} * x * x * x"
                # f" + {coeff[8]} * y * y * y"
                # f" + {coeff[9]} * x * x * x * y * y * y"
            )
            print(cov)
            print(equation)
            return coeff, equation

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, subplot_kw=dict(projection="3d"))

        print()
        print("vel")
        vel_coeff, vel_equation = make_regression(X, y_vel)

        X_reg = np.meshgrid(np.linspace(0, 10, 50), np.linspace(-6, 6, 50))
        ax1.plot_surface(
            X_reg[0],
            X_reg[1],
            f(X_reg, *vel_coeff),
            alpha=0.5,
            label="Velocity regression"
        )
        ax1.scatter(X[0], X[1], y_vel, label="Velocity data")
        ax1.set(xlabel="Distance", ylabel="Radial velocity", zlabel="Velocity")
        ax1.legend()

        print()
        print("angle")
        angle_coeff, angle_equation = make_regression(X, y_angle)

        ax2.plot_surface(
            X_reg[0],
            X_reg[1],
            f(X_reg, *angle_coeff),
            alpha=0.5,
            label="Angle regression"
        )
        ax2.plot_surface(
            X_reg[0],
            X_reg[1],
            np.full_like(X_reg[0], max_angle_allowed),
            alpha=0.5,
            label="Max angle"
        )
        ax2.plot_surface(
            X_reg[0],
            X_reg[1],
            np.full_like(X_reg[0], min_angle_allowed),
            alpha=0.5,
            label="Min angle"
        )
        ax2.scatter(X[0], X[1], y_angle, label="Angle data")
        ax2.set(xlabel="Distance", ylabel="Radial velocity", zlabel="Angle")
        ax2.legend()

        print()
        print("tof")
        tof_coeff, tof_equation = make_regression(X, y_tof)

        ax3.plot_surface(
            X_reg[0],
            X_reg[1],
            f(X_reg, *tof_coeff),
            alpha=0.5,
            label="ToF regression"
        )
        ax3.scatter(X[0], X[1], y_tof, label="ToF data")
        ax3.set(xlabel="Distance", ylabel="Radial velocity", zlabel="ToF")
        ax3.legend()

        with open(dirname(realpath(__file__)) + "/ShootingRegression.java", "w") as f:
            f.write("""package frc.robot.shooting;

/** GENERATED BY shooting_regression.py DO NOT EDIT BY HAND */
public class ShootingRegression {
    public static double calculateVelocityMetersPerSec(double distanceMeters, double radialRobotVelocityMetersPerSec) {
        double x = distanceMeters;
        double y = radialRobotVelocityMetersPerSec;
        return """ + vel_equation + """;
    }

    /** Calculate shot angle **from the horizontal**. Note that hood angle is from the vertical. */
    public static double calculateAngleRad(double distanceMeters, double radialRobotVelocityMetersPerSec) {
        double x = distanceMeters;
        double y = radialRobotVelocityMetersPerSec;
        return """ + angle_equation + """;
    }

    public static double calculateToFSeconds(double distanceMeters, double radialRobotVelocityMetersPerSec) {
        double x = distanceMeters;
        double y = radialRobotVelocityMetersPerSec;
        return """ + tof_equation + """;
    }
}
""")

        plt.show()
