import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

hubx = 2
hubz = 72 * 2.54 / 100
ax.scatter(hubx, 0, hubz, c="red", label="Hub")

t = np.linspace(0, 2, 50)
g = 9.81

def deg_to_rad(deg):
    return deg / 180.0 * np.pi

def rad_to_deg(rad):
    return rad / np.pi * 180.0

def make_shot(v0, vr, vt):
    print(f"v0 = {v0}, vr = {vr}, vt = {vt}")

    # First compute stationary shooting velocity
    discriminant = v0**4 - g * (g * hubx**2 + 2 * hubz * v0**2)
    if discriminant < 0:
        print("\tDiscriminant is negative")
        exit(1)
    phi_1 = np.atan((v0**2 + np.sqrt(discriminant)) / (g * hubx))
    phi_2 = np.atan((v0**2 - np.sqrt(discriminant)) / (g * hubx))

    def is_valid_phi(phi):
        return phi > deg_to_rad(15) and phi < deg_to_rad(90)

    if is_valid_phi(phi_1) and (phi_1 > phi_2 or not is_valid_phi(phi_2)):
        phi = phi_1
    elif is_valid_phi(phi_2):
        phi = phi_2
    else:
        print("\tNo phi found")
        exit(1)

    print(f"\tphi_1 = {rad_to_deg(phi_1)}, phi_2 = {rad_to_deg(phi_2)}, phi = {rad_to_deg(phi)}")

    vx = v0 * np.cos(phi)
    vy = 0
    vz = v0 * np.sin(phi)

    # Now subtract robot velocity from stationary shooting velocity
    vx -= vr
    vy -= vt

    # Now calculate phi, theta, and shooting magnitude from 3d shooting vector
    v = np.sqrt(vx**2 + vy**2 + vz**2)
    phi = np.asin(vz / v)
    theta = np.atan2(vy, vx)
    print(f"\tafter compensate: phi = {rad_to_deg(phi)}, theta = {rad_to_deg(theta)}")

    # For simulation purposes, convert back to 3d velocity vector
    vz = v * np.sin(phi)
    vxy_hypot = v * np.cos(phi)
    vx = vxy_hypot * np.cos(theta)
    vy = vxy_hypot * np.sin(theta)

    s = [vx * t + vr * t, vy * t + vt * t, vz * t + -g * t**2 / 2]

    ax.plot(s[0], s[1], s[2], label=f"$v_0 = {v0}$,\t$v_r = {vr}$,\t$v_t = {vt}$")

make_shot(7, 0, 0)
make_shot(7.5, -1, 0)
make_shot(9, 1, 0)
make_shot(8.5, 0, 1)
make_shot(8, -2, 2)
make_shot(9.5, 0, -1)

ax.set(xlim=[0, hubx + 0.5], ylim=[-1, 1], zlim=[0, hubz + 0.5], xlabel="X", ylabel="Y", zlabel="Z")
ax.legend()

plt.show()

