import numpy as np
import matplotlib.pyplot as plt

# Speed of light
c = 100

# Set brick and observer in rest frame
v_brick = 99
v_obs = 50
s = 0
l = 2
brick_in_rest = {
    'pos': np.array([s,0,0], dtype=np.float32),
    'vel': np.array([v_brick,0,0], dtype=np.float32),
    'size': np.array([l,1,1], dtype=np.float32),
}
observer_in_rest = {
    'pos': np.array([0,0,0], dtype=np.float32),
    'vel': np.array([v_obs,0,0], dtype=np.float32),
}

# Calculate brick [pieces] in observer frame
v_rest_in_obs = -observer_in_rest['vel']
v_brick_in_obs = (v_rest_in_obs + brick_in_rest['vel'])/(1 + v_rest_in_obs*brick_in_rest['vel']/c**2)
gamma_brick_in_obs = 1 / np.sqrt(1 - v_brick_in_obs**2 / c**2)
plane1_in_obs = {
    'pos': brick_in_rest['pos']/gamma_brick_in_obs,
    'vel': v_brick_in_obs,
    'size': np.array([0,1,1]),
}
plane2_in_obs = {
    'pos': (brick_in_rest['pos'] + np.array([brick_in_rest['size'][0],0,0]))/gamma_brick_in_obs,
    'vel': v_brick_in_obs,
    'size': np.array([0,1,1]),
}

# Print info
front_pos = brick_in_rest['pos'] + brick_in_rest['size']
print("--- rest frame ---")
print(f"back of brick: pos = {brick_in_rest['pos'][0]}, vel = {brick_in_rest['vel'][0]}")
print(f"front of brick: pos = {front_pos[0]}, vel = {brick_in_rest['vel'][0]}")
print("--- observer frame ---")
print(f"back of brick: pos = {plane1_in_obs['pos'][0]:.2f}, vel = {plane1_in_obs['vel'][0]:.2f}")
print(f"front of brick: pos = {plane2_in_obs['pos'][0]:.2f}, vel = {plane2_in_obs['vel'][0]:.2f}")

# Define sides of brick
y_range = np.linspace(plane1_in_obs['pos'][1], plane1_in_obs['pos'][1] + plane1_in_obs['size'][1], 2)
z_range = np.linspace(plane1_in_obs['pos'][2], plane1_in_obs['pos'][2] + plane1_in_obs['size'][2], 2)
x_range = np.linspace(plane1_in_obs['pos'][0], plane2_in_obs['pos'][0] - plane1_in_obs['pos'][0], 2)
sq_y, sq_z = np.meshgrid(y_range, z_range)
rc_xy, rc_y = np.meshgrid(x_range, y_range)
rc_xz, rc_z = np.meshgrid(x_range, z_range)
ones = np.ones((2, 2))

# Plot brick
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.plot_surface(rc_xy, rc_y, ones*plane1_in_obs['pos'][2], color='r')
ax.plot_surface(rc_xy, rc_y, ones*(plane1_in_obs['pos'][2] + plane1_in_obs['size'][2]), color='r')
ax.plot_surface(rc_xy, ones*plane1_in_obs['pos'][1], rc_z, color='r')
ax.plot_surface(rc_xy, ones*(plane1_in_obs['pos'][1] + plane1_in_obs['size'][1]), rc_z, color='r')
ax.plot_surface(ones*plane1_in_obs['pos'][0], sq_y, sq_z, color='r')
ax.plot_surface(ones*plane2_in_obs['pos'][0], sq_y, sq_z, color='r')
ax.set_aspect('equal')
plt.show()
