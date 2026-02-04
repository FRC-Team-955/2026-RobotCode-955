import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

hubx = 2
hubz = 72 * 2.54 / 100
ax.scatter(hubx, 0, hubz, c="red", label="Hub")

dt = 0.02
t_final = 2
t = np.linspace(0, t_final, round(t_final / dt))
g = 9.81

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

def calculate_trajectory_kinematics(vel, hood_angle, robot_heading):
    vx, vy, vz = polar_velocity_to_components(vel, hood_angle, robot_heading)

    return (
        vx * t + vr * t,
        vy * t + vt * t,
        vz * t + -g * t ** 2 / 2
    )

def calculate_trajectory_iterative(vel, hood_angle, robot_heading):
    vx, vy, vz = polar_velocity_to_components(vel, hood_angle, robot_heading)

    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = np.zeros_like(t)

    for i in range(len(t)):
        # Get last velocity and position
        lvx = vx
        lvy = vy
        lvz = vz

        if i > 0:
            lx = x[i - 1]
            ly = y[i - 1]
            lz = z[i - 1]
        else:
            lx = ly = lz = 0

        # Integrate acceleration
        vx -= 0.1 * vx * dt
        vz += -g * dt

        # Integrate velocity
        x[i] = lx + (lvx + vx) / 2 * dt
        y[i] = ly + (lvy + vy) / 2 * dt
        z[i] = lz + (lvz + vz) / 2 * dt

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
ax.plot(*calculate_trajectory_iterative(v, hood_angle, robot_heading), label="Iterative")

ax.set(xlim=[0, hubx + 0.5], ylim=[-1, 1], zlim=[0, hubz + 0.5], xlabel="X", ylabel="Y", zlabel="Z")
ax.legend()

plt.show()
