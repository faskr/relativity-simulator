import numpy as np
import matplotlib.pyplot as plt

# Speed of light
c = 100

# Manual settings
v_mov_in_ref_x = c * 0.8
v_obs_in_ref_x = c * 0.4
l_mov_in_mov = np.array([2,1,1])

# Math functions
def mag(v):
    return np.sqrt(np.dot(v, v))

# Projection of u onto v: how long u is in direction of v and which way it's pointing relative to direction of v
def proj(u, v):
    return np.dot(u, v) / np.linalg.norm(v)

# Absolute projection of u onto v: how long u is in direction of v and which way it's pointing in dimension of v
def abs_proj(u, v):
    return proj(u, np.abs(v))

# Velocity addition formula
def v_sum(v_a_in_b, v_b_in_c):
    return (v_a_in_b + v_b_in_c) / (1 + v_a_in_b * v_b_in_c / c**2)

def gamma(v):
    return 1 / np.sqrt(1 - v**2 / c**2)

def contract(x, v):
    return x / gamma(v)

def dilate(x, v):
    return x * gamma(v)

def t_phase(v, s):
    s_mag = proj(s, v)
    return mag(v) * s_mag / c**2

def s_phase(v, t):
    return v * t

def contract_time(t, v, s):
    return contract(t + t_phase(v, s), mag(v))

def dilate_time(t, v, s):
    return dilate(t + t_phase(v, s), mag(v))

def contract_space(s, v, t):
    return contract(s + s_phase(v, t), v)

def dilate_space(s, v, t):
    return dilate(s + s_phase(v, t), v)

# Set mover and observer in their own frames
p1_in_mov = {
    'vel': np.array([0,0,0], dtype=np.float32),
    'pos': np.array([0,0,0], dtype=np.float32),
    'time': 0,
}
p2_in_mov = {
    'vel': p1_in_mov['pos'],
    'pos': np.array([l_mov_in_mov[0],0,0], dtype=np.float32),
    'time': p1_in_mov['time'],
}
obs_in_obs = {
    'vel': np.array([0,0,0], dtype=np.float32),
    'pos': np.array([0,0,0], dtype=np.float32),
    'time': 0,
}

# Set & calculate mover and observer in reference frame
v_p1_in_ref = np.array([v_mov_in_ref_x,0,0], dtype=np.float32)
p1_in_ref = {
    'vel': v_p1_in_ref,
    'pos': contract(p1_in_mov['pos'], v_p1_in_ref),
    'time': dilate(p1_in_mov['time'], mag(v_p1_in_ref)),
}
v_p2_in_ref = np.array([v_mov_in_ref_x,0,0], dtype=np.float32)
p2_in_ref = {
    'vel': v_p2_in_ref,
    'pos': contract(p2_in_mov['pos'], v_p2_in_ref),
    'time': dilate(p2_in_mov['time'], mag(v_p2_in_ref)),
}
v_obs_in_ref = np.array([v_obs_in_ref_x,0,0], dtype=np.float32)
obs_in_ref = {
    'vel': v_obs_in_ref,
    'pos': contract(obs_in_obs['pos'], v_obs_in_ref),
    'time': dilate(obs_in_obs['time'], mag(v_obs_in_ref)),
}

# Calculate mover in observer frame
v_ref_in_obs = -obs_in_ref['vel']
v_p1_in_obs = v_sum(p1_in_ref['vel'], v_ref_in_obs)
v_p2_in_obs = v_sum(p2_in_ref['vel'], v_ref_in_obs)
p1_in_obs = {
    'vel': v_p1_in_obs,
    'pos': contract(p1_in_mov['pos'], v_p1_in_obs),
    'time': dilate(p1_in_mov['time'], mag(v_p1_in_obs)),
}
p2_in_obs = {
    'vel': v_p2_in_obs,
    'pos': contract(p2_in_mov['pos'], v_p2_in_obs),
    'time': dilate(p2_in_mov['time'], mag(v_p2_in_obs)),
}

# Time/phase contraction is analogous to space/length contraction: Find the length of one dimension across a slice of the other, in the rest frame
t_p1_in_mov = 0
t_p1_in_obs = contract_time(t_p1_in_mov, v_p1_in_obs, p1_in_mov['pos'])
t_p2_in_mov = t_p1_in_mov # The mover is simultaneous with itself in its own frame
t_p2_in_obs = contract_time(t_p2_in_mov, v_p2_in_obs, p2_in_mov['pos'])

# Print info
print("--- reference frame ---")
print(f"back of mover: pos = {p1_in_ref['pos'][0]:.2f}, vel = {p1_in_ref['vel'][0]:.2f}")
print(f"front of mover: pos = {p2_in_ref['pos'][0]:.2f}, vel = {p2_in_ref['vel'][0]:.2f}")
print(f"observer: pos = {obs_in_ref['pos'][0]:.2f}, vel = {obs_in_ref['vel'][0]:.2f}")
print("--- mover frame ---")
print(f"back of mover: pos = {p1_in_mov['pos'][0]:.2f}, vel = {p1_in_mov['vel'][0]:.2f}")
print(f"front of mover: pos = {p2_in_mov['pos'][0]:.2f}, vel = {p2_in_mov['vel'][0]:.2f}")
print("--- observer frame ---")
print(f"back of mover: pos = {p1_in_obs['pos'][0]:.2f}, vel = {p1_in_obs['vel'][0]:.2f}")
print(f"front of mover: pos = {p2_in_obs['pos'][0]:.2f}, vel = {p2_in_obs['vel'][0]:.2f}")

# Define coordinates and grids for mover
y_range = np.linspace(p1_in_obs['pos'][1], p1_in_obs['pos'][1] + l_mov_in_mov[1], 2)
z_range = np.linspace(p1_in_obs['pos'][2], p1_in_obs['pos'][2] + l_mov_in_mov[2], 2)
x_range = np.linspace(p1_in_obs['pos'][0], p2_in_obs['pos'][0] - p1_in_obs['pos'][0], 2)
sq_y, sq_z = np.meshgrid(y_range, z_range)
rc_xy, rc_y = np.meshgrid(x_range, y_range)
rc_xz, rc_z = np.meshgrid(x_range, z_range)
ones = np.ones((2, 2))

# Figure
fig = plt.figure()
fig.suptitle('Mover in Observer Frame')

# Plot mover
ax_3d = fig.add_subplot(1,2,1,projection='3d')
ax_3d.plot_surface(rc_xy, rc_y, ones*p1_in_obs['pos'][2], color='r') # z = 0
ax_3d.plot_surface(rc_xy, rc_y, ones*(p1_in_obs['pos'][2] + l_mov_in_mov[2]), color='r') # z = 1
ax_3d.plot_surface(rc_xz, ones*p1_in_obs['pos'][1], rc_z, color='r') # y = 0
ax_3d.plot_surface(rc_xz, ones*(p1_in_obs['pos'][1] + l_mov_in_mov[1]), rc_z, color='r') # y = 1
ax_3d.plot_surface(ones*p1_in_obs['pos'][0], sq_y, sq_z, color='r') # x = 0
ax_3d.plot_surface(ones*p2_in_obs['pos'][0], sq_y, sq_z, color='r') # x = 1
ax_3d.set_xlabel('x')
ax_3d.set_ylabel('y')
ax_3d.set_zlabel('z')
ax_3d.set_aspect('equal')

# Minkowski diagram
ax_md = fig.add_subplot(1,2,2)
x = np.array([p1_in_obs['pos'][0], p2_in_obs['pos'][0]])
t = np.array([c*t_p1_in_obs, c*t_p2_in_obs])
ax_md.plot(x, t, 'r')
ax_md.set_xlabel('x')
ax_md.set_ylabel('t')
ax_md.set_aspect('equal')

# Plot settings
plt.subplots_adjust(wspace=0.5, hspace=0.5)
plt.show()
