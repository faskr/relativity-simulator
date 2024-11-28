import numpy as np
from vector_math import *
from relativity_math import *
from plot_functions import *

# ==== Setup ====

# Manual settings
v_tob_in_hob_x = c * 0.5
v_tib_in_hib_x = c * -0.5
s_sep_in_hob_x = 0 # Start and end point in out-bound home frame
s_tap_in_hob_x = 50 # Turn-around point in out-bound home frame

# ==== Scene ====

### Home Plot

v_hib_in_hob_stationary = np.array([0,0,0], dtype=np.float32) # Relate in-bound journey to out-bound journey by declaring v_hib = v_hob relative to home, tob, and tib, but not traveller
v_hob_in_hib_stationary = -v_hib_in_hob_stationary

## Set Frames

# Out-Bound Journey

# Set out-bound home in its own frame
hob_frame = {}
hob_frame['hob'] = Point([0,0,0], [s_sep_in_hob_x,0,0], 0)

# In-Bound Journey

# Calculate duration of each journey in home frame
t_obj_in_hob = (s_tap_in_hob_x - s_sep_in_hob_x) / v_tob_in_hob_x
s_tap_in_hob = np.array([s_tap_in_hob_x,0,0], dtype=np.float32)
s_sep_in_hob = np.array([s_sep_in_hob_x,0,0], dtype=np.float32)
# Contract, because hib is the foreign frame, because tap and hob are in the same reference frame
s_tap_in_hib_stationary = contract_space(s_tap_in_hob, v_hob_in_hib_stationary, hob_frame['hob'].time)
s_sep_in_hib_stationary = contract_space(s_sep_in_hob, v_hob_in_hib_stationary, hob_frame['hob'].time)
t_ibj_in_hib_stationary = (s_sep_in_hib_stationary[0] - s_tap_in_hib_stationary[0]) / v_tib_in_hib_x

# Set in-bound home in its own frame
hib_frame_stationary = {}
t_hib_in_hib_stationary = contract_time(t_obj_in_hob, v_hob_in_hib_stationary, hob_frame['hob'].pos)
hib_frame_stationary['hib'] = Point([0,0,0], [0,0,0], t_hib_in_hib_stationary)

## Calculate Trajectories

# Calcuate time steps for trajectory calculations
t_steps_ob_in_hob = np.linspace(0, t_obj_in_hob, 100)
t_steps_ib_in_hib_stationary = np.linspace(0, t_ibj_in_hib_stationary, 100)
t_steps_ob_in_home = t_steps_ob_in_hob
t_steps_ib_in_home = t_steps_ib_in_hib_stationary

# Calculate out-bound traveller trajectory in home
v_tob_in_hob = np.array([v_tob_in_hob_x,0,0], dtype=np.float32)
hob_frame['tob'] = Point(v_tob_in_hob, s_sep_in_hob, 0)
traj_tob_in_home = hob_frame['tob'].trajectory(t_steps_ob_in_home)

# Calculate in-bound traveller trajectory in home
v_tib_in_hib = np.array([v_tib_in_hib_x,0,0], dtype=np.float32)
# Continuing to calculate cross-frame coordinates from trajectories, not transforms
# I think this is because transforms disagree with frames about how much time elapsed, so the turnaround point in one frame would be a later point in the other
t_tib_in_hib_stationary = t_hib_in_hib_stationary
hib_frame_stationary['tib'] = Point(v_tib_in_hib, s_tap_in_hib_stationary, t_tib_in_hib_stationary)
traj_tib_in_home = hib_frame_stationary['tib'].trajectory(t_steps_ib_in_home)

# Calculate home trajectories in home
traj_hob_in_home = hob_frame['hob'].trajectory(t_steps_ob_in_home)
traj_hib_in_home = hib_frame_stationary['hib'].trajectory(t_steps_ib_in_home)

## Concatenate trajectories to form paths
path_home_in_home = traj_hob_in_home.concatenate(traj_hib_in_home)
path_trav_in_home = traj_tob_in_home.concatenate(traj_tib_in_home)


### Traveller Plot

# Out-Bound Journey

v_tib_in_tob_trav = np.array([0,0,0], dtype=np.float32)
v_tob_in_tib_trav = -v_tib_in_tob_trav
v_tib_in_hob_trav = v_sum(v_tib_in_tob_trav, v_tob_in_hob)
v_hib_in_tib = -v_tib_in_hib
v_hib_in_hob_trav = np.array(v_sum(v_hib_in_tib, v_tib_in_hob_trav), dtype=np.float32)
v_hob_in_hib_trav = -v_hib_in_hob_trav

# Set out-bound traveller in its own frame
tob_frame = {}
tob_frame['tob'] = Point([0,0,0], [0,0,0], 0)

# Calculate out-bound journey of home in traveller frame
v_hob_in_tob = -v_tob_in_hob
# Not sure if contraction is correct here, or why it is
s_hob_in_tob = contract_space(hob_frame['hob'].pos, v_hob_in_tob, hob_frame['hob'].time)
t_hob_in_tob = contract_time(hob_frame['hob'].time, v_hob_in_tob, hob_frame['hob'].pos)
tob_frame['hob'] = Point(v_hob_in_tob, s_hob_in_tob, t_hob_in_tob)
#tob_frame['hob'] = hob_frame['hob'].new_frame(v_hob_in_tob)

# Calculate pos and time of traveller's turnaround point; pos is relative to traveller at the beginning of its journey
s_tap_in_tob = contract_space(s_tap_in_hob, v_hob_in_tob, hob_frame['tob'].time)
t_tap_in_tob = (s_tap_in_tob[0] - tob_frame['tob'].pos[0]) / -v_hob_in_tob[0]
#t_tap_in_tob = contract_time(t_tap_in_hob, v_hob_in_tob, hob_frame['tob'].pos)

# Calculate time steps for trajectory calculations
t_steps_tob_in_tob = np.linspace(0, t_tap_in_tob, 100)
t_steps_tib_in_tib = t_steps_tob_in_tob

# Calculate out-bound trajectories in traveller frame
traj_hob_in_tob = tob_frame['hob'].trajectory(t_steps_tob_in_tob)

# Set in-bound traveller in its own frame
tib_frame = {}
tib_frame['tib'] = Point([0,0,0], [0,0,0], t_tap_in_tob)

# Calculate in-bound home journey in traveller frame
v_hib_in_tib = -v_tib_in_hib
tib_frame['hib'] = Point(v_hib_in_tib, traj_hob_in_tob.pos[-1,:], traj_hob_in_tob.time[-1])
traj_hib_in_tib = tib_frame['hib'].trajectory(t_steps_tib_in_tib)

# Set and calculate traveller trajectories in traveller frame
traj_tob_in_tob = tob_frame['tob'].trajectory(t_steps_tob_in_tob)
traj_tib_in_tib = tib_frame['tib'].trajectory(t_steps_tib_in_tib)

## Concatenate Trajectories to form Paths; need to revise this so that all variables are in traveller frame, not tob or tib
path_home_in_trav = traj_hob_in_tob.concatenate(traj_hib_in_tib)
path_trav_in_trav = traj_tob_in_tob.concatenate(traj_tib_in_tib)


### Out-Bound Traveller Plot

v_hib_in_tob = v_sum(v_hib_in_hob_stationary, v_hob_in_tob)
tob_frame['hib'] = Point(v_hib_in_tob, traj_hob_in_tob.pos[-1,:], traj_hob_in_tob.time[-1])
traj_hib_in_tob = tob_frame['hib'].trajectory(t_steps_tib_in_tib)
v_tib_in_hob = v_sum(v_tib_in_hib, v_hib_in_hob_stationary)
v_tib_in_tob = v_sum(v_tib_in_hob, v_hob_in_tob)
tob_frame['tib'] = Point(v_tib_in_tob, traj_tob_in_tob.pos[-1,:], traj_tob_in_tob.time[-1])
traj_tib_in_tob = tob_frame['tib'].trajectory(t_steps_tib_in_tib)
path_home_in_tob = traj_hob_in_tob.concatenate(traj_hib_in_tob)
path_trav_in_tob = traj_tob_in_tob.concatenate(traj_tib_in_tob)


### In-Bound Traveller Plot

#v_tob_in_tib = -v_tib_in_tob
#v_hob_in_tib = v_sum(v_hob_in_tob, v_tob_in_tib)
#tib_frame['hob'] = Point(v_hob_in_tib, )

# ==== Output ====

# Plot journeys in home frame
fig = plt.figure()
fig.suptitle('Twin Trajectories')

ax_journey = fig.add_subplot(2,2,1)
ax_journey.set_title('Journey in Home Frame')
plot_spacetime(ax_journey, (path_home_in_home, path_trav_in_home), do_cones=False)

ax_journey = fig.add_subplot(2,2,2)
ax_journey.set_title('Journey in Traveller Frame')
plot_spacetime(ax_journey, (path_home_in_trav, path_trav_in_trav), do_cones=False)

ax_journey = fig.add_subplot(2,2,3)
ax_journey.set_title('Journey in Out-Bound Traveller Frame')
plot_spacetime(ax_journey, (path_home_in_tob, path_trav_in_tob), do_cones=False)

# Plot settings
plt.subplots_adjust(wspace=0, hspace=0.3)
plt.show()
