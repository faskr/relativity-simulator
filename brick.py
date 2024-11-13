import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.path as path

# Speed of light
c = 100

# Manual settings
v_brick_in_ref_x = c*0.5
v_obs_in_ref_x = c*0

def mag(v):
    return np.sqrt(np.dot(v, v))

def gamma(v):
    return 1 / np.sqrt(1 - v**2 / c**2)

def v_sum(v_a_in_b, v_b_in_c):
    return (v_a_in_b + v_b_in_c) / (1 + v_a_in_b*v_b_in_c / c**2)

# Set brick and observer in their own frames
l_brick_in_brick_x = 2
brick_in_brick = {
    'pos': np.array([0,0,0], dtype=np.float32),
    'vel': np.array([0,0,0], dtype=np.float32),
    'time': 1,
    'size': np.array([l_brick_in_brick_x,1,1], dtype=np.float32),
}
obs_in_obs = {
    'pos': np.array([0,0,0], dtype=np.float32),
    'vel': np.array([0,0,0], dtype=np.float32),
    'time': 1,
}
# Set & calculate brick and observer in reference frame
v_brick_in_ref = np.array([v_brick_in_ref_x,0,0], dtype=np.float32)
gamma_brick_in_ref = gamma(v_brick_in_ref)
s_brick_in_ref = brick_in_brick['pos'] / gamma_brick_in_ref
l_brick_in_ref_x = l_brick_in_brick_x / gamma_brick_in_ref[0]
gamma_brick_in_ref_mag = gamma(mag(v_brick_in_ref))
t_brick_in_ref = brick_in_brick['time'] * gamma_brick_in_ref_mag

# debug
assert gamma(mag(v_brick_in_ref)) - gamma_brick_in_ref[0] < 0.0001
#

brick_in_ref = {
    'pos': s_brick_in_ref,
    'vel': v_brick_in_ref,
    'time': t_brick_in_ref,
    'size': np.array([l_brick_in_ref_x,0,0], dtype=np.float32),
}
v_obs_in_ref = np.array([v_obs_in_ref_x,0,0], dtype=np.float32)
gamma_obs_in_ref_mag = gamma(mag(v_obs_in_ref))
t_obs_in_ref = obs_in_obs['time'] * gamma_obs_in_ref_mag
obs_in_ref = {
    'pos': np.array([0,0,0], dtype=np.float32),
    'vel': np.array([v_obs_in_ref_x,0,0], dtype=np.float32),
    'time': t_obs_in_ref,
}
# Calculate brick [pieces] in observer frame
v_ref_in_obs = -obs_in_ref['vel']
v_brick_in_obs = v_sum(brick_in_ref['vel'], v_ref_in_obs)
gamma_brick_in_obs = gamma(v_brick_in_obs)
s_brick_in_obs = brick_in_brick['pos'] / gamma_brick_in_obs
size_brick_in_obs = brick_in_brick['size'] / gamma_brick_in_obs
v_brick_in_obs_mag = mag(v_brick_in_obs)
gamma_brick_in_obs_mag = gamma(v_brick_in_obs_mag)
t_brick_in_obs = brick_in_brick['time'] * gamma_brick_in_obs_mag

#debug
t_pre_mag = brick_in_brick['time'] * gamma(mag(v_brick_in_obs))
assert t_pre_mag - t_brick_in_obs < 0.0001
#

plane1_in_obs = {
    'pos': s_brick_in_obs,
    'vel': v_brick_in_obs,
    'time': t_brick_in_obs,
    'size': np.array([0,1,1]),
}
plane2_in_obs = {
    'pos': s_brick_in_obs + size_brick_in_obs,
    'vel': v_brick_in_obs,
    'time': t_brick_in_obs,
    'size': np.array([0,1,1]),
}

s_brick_in_brick_mag = np.dot(brick_in_brick['pos'], np.abs(v_brick_in_obs)) / np.linalg.norm(-v_brick_in_obs)
size_brick_in_brick_mag = np.dot(brick_in_brick['size'], np.abs(v_brick_in_obs)) / np.linalg.norm(-v_brick_in_obs)
# "Correct" formula
t_abs_plane1_in_obs = (0 + v_brick_in_obs_mag * s_brick_in_brick_mag / c**2) * gamma_brick_in_obs_mag
# The correct Lorentz tf not only creates an unexpected result that doesn't fit with geogebra's simultaneity axis slopes, but plugging in and calculating t and t' repeatedly yields different results - the formulas do not reverse each other
# Perhaps there are two formulas because the moving frame could move in either direction relative to the rest frame
# Or maybe each frame sees the other's line of simultaneous events as being more slanted/ahead than its own, just like each sees the other's space as being more compressed
# Look at Lorentz transformation Wiki; maybe length contraction needs to use a more complicated formula and there needs to be some elapsed time? that might help repeated calculation
# "Incorrect" formula with correct result, according to geogebra axis slope and based on expected plot output
s_plane2_in_brick_mag = s_brick_in_brick_mag + size_brick_in_brick_mag
t_abs_plane2_in_obs = (v_brick_in_obs_mag * s_plane2_in_brick_mag / c**2) / gamma_brick_in_obs_mag
# But this is the correct translation to observer coordinates for [0, 3.464] according to geogebra; it makes no sense
#print(c*(3.464/c + (v_brick_in_obs_mag * 0 / c**2)) * gamma_brick_in_obs_mag)

#print(s_brick_in_brick_mag) # = 0
#print(size_brick_in_brick_mag) # = 2
#print(size_brick_in_obs_mag) # = 1.732
#print(v_brick_in_obs[0]) # = 50
#print(gamma_brick_in_obs_mag) # = 1.155
# Correct formula with correct result but wrong direction
#s_brick_in_obs_mag = np.dot(plane1_in_obs['pos'], np.abs(v_brick_in_obs)) / np.linalg.norm(-v_brick_in_obs)
#size_brick_in_obs_mag = np.dot(plane2_in_obs['pos'], np.abs(v_brick_in_obs)) / np.linalg.norm(-v_brick_in_obs)
#t_abs_plane2_in_brick = (0 - v_brick_in_obs_mag * (s_brick_in_obs_mag + size_brick_in_obs_mag) / c**2) * gamma_brick_in_obs_mag # i.e. "t=0 at p2 in obs translates to ANS"
# "Correct" formula in the correct direction but with incorrect result
#t_abs_plane2_in_obs = (0 + v_brick_in_obs_mag * (s_brick_in_brick_mag + size_brick_in_brick_mag) / c**2) * gamma_brick_in_obs_mag # i.e. "t=0 at p2 in brick translates to ANS"
# If that formula is correct, and if s_brick_in_obs = 1.732 (length contraction) means that c*t_abs_plane2_in_obs = 0.866 (geogebra), then size_brick_in_brick = 1.1196, which it doesn't
# Perhaps the formula is only meant to translate from obs to brick, not brick to obs; tho a document showed it as translating from brick to obs
#print(c*t_abs_plane2_in_brick) # = -1
#print(c*t_abs_plane2_in_obs) # should be 0.866

# Print info
front_pos_in_ref = brick_in_ref['pos'] + brick_in_ref['size']
front_pos_in_brick = brick_in_brick['pos'] + brick_in_brick['size']
print("--- reference frame ---")
print(f"back of brick: pos = {brick_in_ref['pos'][0]:.2f}, vel = {brick_in_ref['vel'][0]:.2f}")
print(f"front of brick: pos = {front_pos_in_ref[0]:.2f}, vel = {brick_in_ref['vel'][0]:.2f}")
print(f"observer: pos = {obs_in_ref['pos'][0]:.2f}, vel = {obs_in_ref['vel'][0]:.2f}")
print("--- brick frame ---")
print(f"back of brick: pos = {brick_in_brick['pos'][0]:.2f}, vel = {brick_in_brick['vel'][0]:.2f}")
print(f"front of brick: pos = {front_pos_in_brick[0]:.2f}, vel = {brick_in_brick['vel'][0]:.2f}")
print("--- observer frame ---")
print(f"back of brick: pos = {plane1_in_obs['pos'][0]:.2f}, vel = {plane1_in_obs['vel'][0]:.2f}")
print(f"front of brick: pos = {plane2_in_obs['pos'][0]:.2f}, vel = {plane2_in_obs['vel'][0]:.2f}")

# Define coordinates and grids for brick
y_range = np.linspace(plane1_in_obs['pos'][1], plane1_in_obs['pos'][1] + plane1_in_obs['size'][1], 2)
z_range = np.linspace(plane1_in_obs['pos'][2], plane1_in_obs['pos'][2] + plane1_in_obs['size'][2], 2)
x_range = np.linspace(plane1_in_obs['pos'][0], plane2_in_obs['pos'][0] - plane1_in_obs['pos'][0], 2)
sq_y, sq_z = np.meshgrid(y_range, z_range)
rc_xy, rc_y = np.meshgrid(x_range, y_range)
rc_xz, rc_z = np.meshgrid(x_range, z_range)
ones = np.ones((2, 2))

# Plot brick
fig = plt.figure()
ax_3d = fig.add_subplot(1,2,1,projection='3d')
ax_3d.plot_surface(rc_xy, rc_y, ones*plane1_in_obs['pos'][2], color='r') # z = 0
ax_3d.plot_surface(rc_xy, rc_y, ones*(plane1_in_obs['pos'][2] + plane1_in_obs['size'][2]), color='r') # z = 1
ax_3d.plot_surface(rc_xz, ones*plane1_in_obs['pos'][1], rc_z, color='r') # y = 0
ax_3d.plot_surface(rc_xz, ones*(plane1_in_obs['pos'][1] + plane1_in_obs['size'][1]), rc_z, color='r') # y = 1
ax_3d.plot_surface(ones*plane1_in_obs['pos'][0], sq_y, sq_z, color='r') # x = 0
ax_3d.plot_surface(ones*plane2_in_obs['pos'][0], sq_y, sq_z, color='r') # x = 1
ax_3d.set_aspect('equal')

# Simultaneity graph
ax_md = fig.add_subplot(1,2,2)
x = np.array([plane1_in_obs['pos'][0], plane2_in_obs['pos'][0]])
t = np.array([c*t_abs_plane1_in_obs, c*t_abs_plane2_in_obs])
ax_md.plot(x, t, 'r')
ax_md.set_title('Simultaneity of Brick in Observer Frame')
ax_md.set_xlabel('Location')
ax_md.set_ylabel('Time')
ax_md.set_aspect('equal')

plt.show()
