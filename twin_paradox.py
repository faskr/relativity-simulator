import numpy as np
from vector_math import *
from relativity_math import *
from plot_functions import *

# ==== Setup ====

# Manual settings
v_tob_in_hob_x = c * 0.5
v_tib_in_hib_x = c * -0.5
t_turnaround_home = 1
# TODO: turnaround point should eventually be based on location, not time
#s_turnaround_home_x = 50
#t_turnaround_home = s_turnaround_home_x / v_tob_in_hob_x
t_steps_ob = np.linspace(0, t_turnaround_home, 100)
t_steps_ib = np.linspace(0, t_turnaround_home, 100)

# ==== Scene ====

# Set out-bound home and traveller in their own frames
hob_frame = {}
tob_frame = {}
hob_frame['hob'] = create_point([0,0,0], [0,0,0], 0)
tob_frame['tob'] = create_point([0,0,0], [0,0,0], 0)

# Set and calculate out-bound journey of home and traveller in both frames
traj_hob_in_hob = trajectory(hob_frame['hob'], t_steps_ob)
traj_tob_in_tob = trajectory(tob_frame['tob'], t_steps_ob)
v_tob_in_hob = np.array([v_tob_in_hob_x,0,0], dtype=np.float32)
# Calculate cross-frame coordinates from transforms, then from trajectories
# I don't know whether to keep this or not
hob_frame['tob'] = point_in_new_frame(v_tob_in_hob, tob_frame['tob'])
traj_tob_in_hob = trajectory(hob_frame['tob'], t_steps_ob)
v_hob_in_tob = np.array([-v_tob_in_hob_x,0,0], dtype=np.float32)
tob_frame['hob'] = point_in_new_frame(v_hob_in_tob, hob_frame['hob'])
traj_hob_in_tob = trajectory(tob_frame['hob'], t_steps_ob)

# Set in-bound home and traveller in their own frames
hib_frame = {}
tib_frame = {}
hib_frame['hib'] = create_point([0,0,0], [0,0,0], t_turnaround_home)
tib_frame['tib'] = create_point([0,0,0], [0,0,0], t_turnaround_home)

# Set and calculate in-bound journey of home and traveller in both frames
traj_hib_in_hib = trajectory(hib_frame['hib'], t_steps_ib)
traj_tib_in_tib = trajectory(tib_frame['tib'], t_steps_ib)
v_tib_in_hib = np.array([v_tib_in_hib_x,0,0], dtype=np.float32)
# Continue to calculate cross-frame coordinates from trajectories, not transforms
# I think this is because transforms disagree with frames about how much time elapsed, so the turnaround point in one frame would be a later point in the other
hib_frame['tib'] = create_point(v_tib_in_hib, traj_tob_in_hob['pos'][-1,:], traj_tob_in_hob['time'][-1])
traj_tib_in_hib = trajectory(hib_frame['tib'], t_steps_ib)
v_hib_in_tib = np.array([-v_tib_in_hib_x,0,0], dtype=np.float32)
tib_frame['hib'] = create_point(v_hib_in_tib, traj_hob_in_tob['pos'][-1,:], traj_hob_in_tob['time'][-1])
traj_hib_in_tib = trajectory(tib_frame['hib'], t_steps_ib)

# Plot out-bound journeys in home frame
fig = plt.figure()
fig.suptitle('Twin Trajectories')

ax_home_ob = fig.add_subplot(1,2,1)
ax_home_ob.set_title('Out-Bound Journey in Home Frame')
plot_spacetime(ax_home_ob, (traj_hob_in_hob, traj_tob_in_hob), do_cones=False)

ax_home_ib = fig.add_subplot(1,2,2)
ax_home_ib.set_title('In-Bound Journey in Home Frame')
plot_spacetime(ax_home_ib, (traj_hib_in_hib, traj_tib_in_hib), do_cones=False)

# Plot settings
plt.subplots_adjust(wspace=0.5, hspace=0.5)
plt.show()
