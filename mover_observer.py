import numpy as np
from vector_math import *
from relativity import *
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
mov_frame['p1'] = Point([0,0,0], [0,0,0], 0)
mov_frame['p2'] = Point(mov_frame['p1'].vel, [l_mov_in_mov[0],0,0], mov_frame['p1'].time)
obs_frame['obs'] = Point([0,0,0], [0,0,0], 0)

# Set & calculate mover and observer in reference frame
v_p1_in_ref = np.array([v_mov_in_ref_x,0,0], dtype=np.float32)
v_p2_in_ref = np.array([v_mov_in_ref_x,0,0], dtype=np.float32)
v_obs_in_ref = np.array([v_obs_in_ref_x,0,0], dtype=np.float32)
ref_frame = {}
ref_frame['p1'] = mov_frame['p1'].new_frame(v_p1_in_ref)
ref_frame['p2'] = mov_frame['p2'].new_frame(v_p2_in_ref)
ref_frame['obs'] = obs_frame['obs'].new_frame(v_obs_in_ref)

# Calculate mover in observer frame
v_ref_in_obs = -ref_frame['obs'].vel
v_p1_in_obs = v_transform(ref_frame['p1'].vel, v_ref_in_obs)
v_p2_in_obs = v_transform(ref_frame['p2'].vel, v_ref_in_obs)
obs_frame['p1'] = mov_frame['p1'].new_frame(v_p1_in_obs)
obs_frame['p2'] = mov_frame['p2'].new_frame(v_p2_in_obs)
# Length of the mover in a simultaneous slice of time in the observer frame
l_mov_in_obs_x = contract(mov_frame['p2'].pos[0] - mov_frame['p1'].pos[0], obs_frame['p1'].vel[0])

# ==== Output ====

# Print info
print("--- reference frame ---")
print(f"back of mover: vel = {ref_frame['p1'].vel[0]:.2f}, pos = {ref_frame['p1'].pos[0]:.2f}, c*time = {c*ref_frame['p1'].time:.2f}")
print(f"front of mover: vel = {ref_frame['p2'].vel[0]:.2f}, pos = {ref_frame['p2'].pos[0]:.2f}, c*time = {c*ref_frame['p2'].time:.2f}")
print(f"observer: vel = {ref_frame['obs'].vel[0]:.2f}, pos = {ref_frame['obs'].pos[0]:.2f}, c*time = {c*ref_frame['obs'].time:.2f}")
print("--- mover frame ---")
print(f"back of mover: vel = {mov_frame['p1'].vel[0]:.2f}, pos = {mov_frame['p1'].pos[0]:.2f}, c*time = {c*mov_frame['p1'].time:.2f}")
print(f"front of mover: vel = {mov_frame['p2'].vel[0]:.2f}, pos = {mov_frame['p2'].pos[0]:.2f}, c*time = {c*mov_frame['p2'].time:.2f}")
print("--- observer frame ---")
print(f"back of mover: vel = {obs_frame['p1'].vel[0]:.2f}, pos = {obs_frame['p1'].pos[0]:.2f}, c*time = {c*obs_frame['p1'].time:.2f}")
print(f"front of mover: vel = {obs_frame['p2'].vel[0]:.2f}, pos = {obs_frame['p2'].pos[0]:.2f}, c*time = {c*obs_frame['p2'].time:.2f}")

# Plot info
create_relativity_subplots(obs_frame['p1'], obs_frame['p2'], [l_mov_in_obs_x, l_mov_in_mov[1], l_mov_in_mov[2]], t_steps_obs)
