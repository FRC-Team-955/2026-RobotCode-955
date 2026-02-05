import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

# All length quantities are in meters

hubx = 2
hubz = 72 * 2.54 / 100
ax.scatter(hubx, 0, hubz, c="red", label="Hub")

dt = 0.005
t_final = 2
t = np.linspace(0, t_final, round(t_final / dt))

g = 9.81
fuel_mass = 0.2150028  # kg - note, this is the average weight according to the range in the game manual
fuel_radius = (15 / 100) / 2
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

def polar_velocity_to_components(vel, pitch, yaw):
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

def calculate_trajectory_kinematics(vel, hood_angle, robot_heading):
    vx, vy, vz = polar_velocity_to_components(vel, hood_angle, robot_heading)

    return (
        vx * t + vr * t,
        vy * t + vt * t,
        vz * t + (1 / 2) * -g * t ** 2
    )

def calculate_trajectory_iterative(vel, hood_angle, robot_heading, apply_magnus_effect):
    vx, vy, vz = polar_velocity_to_components(vel, hood_angle, robot_heading)

    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = np.zeros_like(t)
    i_end = None
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
        if apply_magnus_effect:
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

        if z[i] > hubz:
            past_hub_on_upwards_arc = True
        elif z[i] < hubz and past_hub_on_upwards_arc:
            i_end = i + 1
            break

    if i_end is not None:
        return x[:i_end], y[:i_end], z[:i_end]
    else:
        return x, y, z

v0 = 9
vr = 0
vt = 0

print(f"v0 = {v0}, vr = {vr}, vt = {vt}")

# First compute stationary shooting velocity
discriminant = v0 ** 4 - g * (g * hubx ** 2 + 2 * hubz * v0 ** 2)
if discriminant < 0:
    print("\tDiscriminant is negative")
    exit(1)
phi_1 = np.atan((v0 ** 2 + np.sqrt(discriminant)) / (g * hubx))
phi_2 = np.atan((v0 ** 2 - np.sqrt(discriminant)) / (g * hubx))

def is_valid_phi(phi):
    return deg_to_rad(15) < phi < deg_to_rad(90)

if is_valid_phi(phi_1) and (phi_1 > phi_2 or not is_valid_phi(phi_2)):
    hood_angle = phi_1
elif is_valid_phi(phi_2):
    hood_angle = phi_2
else:
    print("\tNo phi found")
    exit(1)

print(f"\tphi_1 = {rad_to_deg(phi_1)}, phi_2 = {rad_to_deg(phi_2)}, hood_angle = {rad_to_deg(hood_angle)}")

vx = v0 * np.cos(hood_angle)
vy = 0
vz = v0 * np.sin(hood_angle)

# Now subtract robot velocity from stationary shooting velocity
vx -= vr
vy -= vt

# Now calculate hood_angle, robot_heading, and shooting magnitude from 3d shooting vector
v = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
hood_angle = np.asin(vz / v)
robot_heading = np.atan2(vy, vx)
print(
    f"\tafter compensate: v = {v}, hood_angle = {rad_to_deg(hood_angle)}, robot_heading = {rad_to_deg(robot_heading)}")

ax.plot(*calculate_trajectory_kinematics(v, hood_angle, robot_heading), label="Kinematics")
ax.plot(*calculate_trajectory_iterative(v, hood_angle, robot_heading, False), label="Iterative (w/o magnus effect)")
# ax.plot(*calculate_trajectory_iterative(v, hood_angle, robot_heading, True), label="Iterative (w/ magnus effect)")

ax.set(xlim=[0, hubx + 0.5], ylim=[-1, 1], zlim=[0, hubz + 2], xlabel="X", ylabel="Y", zlabel="Z")
ax.legend()

plt.show()
