import numpy as np
import matplotlib.pyplot as plt
from vector_math import *
from relativity_math import *
from plot_functions import *

# ==== Setup ====

# Manual settings
v_mov_in_ref_x = c * 0.8
v_obs_in_ref_x = c * 0.4
l_mov_in_mov = np.array([2,1,1])
t_steps_obs = np.linspace(0, 0.04, 5)

# ==== Scene ====

# Set mover and observer in their own frames
mov_frame = {}
obs_frame = {}
mov_frame['p1'] = create_point([0,0,0], [0,0,0], 0)
mov_frame['p2'] = create_point(mov_frame['p1']['pos'], [l_mov_in_mov[0],0,0], mov_frame['p1']['time'])
obs_frame['obs'] = create_point([0,0,0], [0,0,0], 0)

# Set & calculate mover and observer in reference frame
v_p1_in_ref = np.array([v_mov_in_ref_x,0,0], dtype=np.float32)
v_p2_in_ref = np.array([v_mov_in_ref_x,0,0], dtype=np.float32)
v_obs_in_ref = np.array([v_obs_in_ref_x,0,0], dtype=np.float32)
ref_frame = {}
ref_frame['p1'] = point_in_new_frame(v_p1_in_ref, mov_frame, 'p1')
ref_frame['p2'] = point_in_new_frame(v_p2_in_ref, mov_frame, 'p2')
ref_frame['obs'] = point_in_new_frame(v_obs_in_ref, obs_frame, 'obs')

# Calculate mover in observer frame
v_ref_in_obs = -ref_frame['obs']['vel']
v_p1_in_obs = v_sum(ref_frame['p1']['vel'], v_ref_in_obs)
v_p2_in_obs = v_sum(ref_frame['p2']['vel'], v_ref_in_obs)
obs_frame['p1'] = point_in_new_frame(v_p1_in_obs, mov_frame, 'p1')
obs_frame['p2'] = point_in_new_frame(v_p2_in_obs, mov_frame, 'p2')
# Length of the mover in a simultaneous slice of time in the observer frame
l_mov_in_obs_x = contract(mov_frame['p2']['pos'][0] - mov_frame['p1']['pos'][0], obs_frame['p1']['vel'][0])

# ==== Output ====

# Print info
print("--- reference frame ---")
print(f"back of mover: vel = {ref_frame['p1']['vel'][0]:.2f}, pos = {ref_frame['p1']['pos'][0]:.2f}, c*time = {c*ref_frame['p1']['time']:.2f}")
print(f"front of mover: vel = {ref_frame['p2']['vel'][0]:.2f}, pos = {ref_frame['p2']['pos'][0]:.2f}, c*time = {c*ref_frame['p2']['time']:.2f}")
print(f"observer: vel = {ref_frame['obs']['vel'][0]:.2f}, pos = {ref_frame['obs']['pos'][0]:.2f}, c*time = {c*ref_frame['obs']['time']:.2f}")
print("--- mover frame ---")
print(f"back of mover: vel = {mov_frame['p1']['vel'][0]:.2f}, pos = {mov_frame['p1']['pos'][0]:.2f}, c*time = {c*mov_frame['p1']['time']:.2f}")
print(f"front of mover: vel = {mov_frame['p2']['vel'][0]:.2f}, pos = {mov_frame['p2']['pos'][0]:.2f}, c*time = {c*mov_frame['p2']['time']:.2f}")
print("--- observer frame ---")
print(f"back of mover: vel = {obs_frame['p1']['vel'][0]:.2f}, pos = {obs_frame['p1']['pos'][0]:.2f}, c*time = {c*obs_frame['p1']['time']:.2f}")
print(f"front of mover: vel = {obs_frame['p2']['vel'][0]:.2f}, pos = {obs_frame['p2']['pos'][0]:.2f}, c*time = {c*obs_frame['p2']['time']:.2f}")

# Figure
fig = plt.figure()
fig.suptitle('Mover in Observer Frame')

# Plot mover
ax_3d = fig.add_subplot(1,2,1,projection='3d')
ax_3d.set_title('Appearance')
plot_3d_box(ax_3d, obs_frame['p1']['pos'], [l_mov_in_obs_x, l_mov_in_mov[1], l_mov_in_mov[2]])

# Minkowski spacetime
ax_md = fig.add_subplot(1,2,2)
ax_md.set_title('Minkowski Spacetime')
create_spacetime_plot(obs_frame['p1'], obs_frame['p2'], t_steps_obs, ax_md)

# Plot settings
plt.subplots_adjust(wspace=0.5, hspace=0.5)
plt.grid()
plt.show()
