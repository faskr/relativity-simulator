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

# Velocity difference between out-bound and in-bound sections of home frame
v_hib_in_hob_stationary = np.array([0,0,0], dtype=np.float32) # Relate in-bound journey to out-bound journey by declaring v_hib = v_hob relative to home, tob, and tib, but not traveller
v_hob_in_hib_stationary = -v_hib_in_hob_stationary

# Duration of out-bound journey in home frame
t_obj_in_hob = (s_tap_in_hob_x - s_sep_in_hob_x) / v_tob_in_hob_x

# Out-bound home point in home frame
hob_frame = {}
hob_frame['hob'] = Point([0,0,0], [s_sep_in_hob_x,0,0], 0)

# Duration of in-bound journey in home frame
s_tap_in_hob = np.array([s_tap_in_hob_x,0,0], dtype=np.float32)
s_sep_in_hob = np.array([s_sep_in_hob_x,0,0], dtype=np.float32)
# Contract (supposing v_hob_in_hib =/= 0):
#   s_tap and s_sep are defined in hob, but measured simultaneously in hib, i.e. s_sep and s_tap are lengths
#   t_ibj in hob is defined in hob, but is then measured at a point in space that is the same in hib throughout the measurement, i.e. t_ibj is a presence duration
s_tap_in_hib_stationary = contract_space(s_tap_in_hob, v_hob_in_hib_stationary, hob_frame['hob'].time)
s_sep_in_hib_stationary = contract_space(s_sep_in_hob, v_hob_in_hib_stationary, hob_frame['hob'].time)
t_ibj_in_hib_stationary = (s_sep_in_hib_stationary[0] - s_tap_in_hib_stationary[0]) / v_tib_in_hib_x

t_hob_in_hob = 0
v_zero = np.array([0,0,0], dtype=np.float32)
path_home_in_home = Path.create_path(s_sep_in_hob, t_hob_in_hob, [t_obj_in_hob, t_ibj_in_hib_stationary], [v_zero, v_zero])
t_tob_in_hob = 0
v_tob_in_hob = np.array([v_tob_in_hob_x,0,0], dtype=np.float32)
v_tib_in_hib = np.array([v_tib_in_hib_x,0,0], dtype=np.float32)
path_trav_in_home = Path.create_path(s_sep_in_hob, t_tob_in_hob, [t_obj_in_hob, t_ibj_in_hib_stationary], [v_tob_in_hob, v_tib_in_hib])

# TODO: Generalize these comments about the origin of acceleration and put them somewhere for reference; it's a useful concept
# Length and presence contraction: sep and tap are defined in hob; in a simultaneous point in time in hib, sep and tap are closer to the origin of acceleration (i.e. where acceleration is observed)
# In any given [space/time] instant in hib, sep and tap are closer to the origin and each other [in time/space] (if not the same distance) than in any given instant in hob


### Traveller Plot

v_tib_in_tob_trav = np.array([0,0,0], dtype=np.float32)
v_tob_in_tib_trav = -v_tib_in_tob_trav
# Not sure if these are needed
v_tib_in_hob_trav = v_sum(v_tib_in_tob_trav, v_tob_in_hob)
v_hib_in_tib = -v_tib_in_hib
v_hib_in_hob_trav = np.array(v_sum(v_hib_in_tib, v_tib_in_hob_trav), dtype=np.float32)
v_hob_in_hib_trav = -v_hib_in_hob_trav

# Out-Bound Journey

# Calculate pos and time of traveller's turnaround point; pos is relative to traveller at the beginning of its journey
v_hob_in_tob = -v_tob_in_hob
t_obj_in_tob = contract_time(t_obj_in_hob, v_hob_in_tob, s_sep_in_hob)
#t_obj_in_tob = (s_tap_in_tob[0] - s_sep_in_tob[0]) / -v_hob_in_tob[0]

# Set out-bound traveller in its own frame
tob_frame = {}
tob_frame['tob'] = Point([0,0,0], [0,0,0], 0)

# In-Bound Journey

s_tap_in_tob = contract_space(s_tap_in_hob, v_hob_in_tob, s_sep_in_hob)
s_tap_in_tib_trav = contract_space(s_tap_in_tob, v_tob_in_tib_trav, tob_frame['tob'].time) # TODO: in tib, tap is 0 and sep is negative
s_sep_in_tob = s_sep_in_hob # TODO: use v_tob_in_hob
s_sep_in_tib_trav = contract_space(s_sep_in_tob, v_tob_in_tib_trav, tob_frame['tob'].time)
t_ibj_in_tib_trav = (s_sep_in_tib_trav[0] - s_tap_in_tib_trav[0]) / v_tib_in_hib_x

# Set in-bound traveller in its own frame
tib_frame_trav = {}
t_tib_in_tib_trav = contract_time(t_obj_in_tob, v_tob_in_tib_trav, tob_frame['tob'].pos)
tib_frame_trav['tib'] = Point([0,0,0], [0,0,0], t_tib_in_tib_trav)

# Trajectories

# Calculate time steps for trajectory calculations
t_steps_ob_in_tob = np.linspace(0, t_obj_in_tob, 100)
t_steps_ib_in_tib_trav = np.linspace(0, t_ibj_in_tib_trav, 100)
t_steps_ob_in_trav = t_steps_ob_in_tob
t_steps_ib_in_trav = t_steps_ib_in_tib_trav

# Calculate out-bound journey of home in traveller frame
s_hob_in_tob = contract_space(hob_frame['hob'].pos, v_hob_in_tob, hob_frame['hob'].time)
t_hob_in_tob = contract_time(hob_frame['hob'].time, v_hob_in_tob, hob_frame['hob'].pos)
tob_frame['hob'] = Point(v_hob_in_tob, s_hob_in_tob, t_hob_in_tob)
#tob_frame['hob'] = hob_frame['hob'].new_frame(v_hob_in_tob)

# Calculate out-bound trajectory in traveller frame
traj_hob_in_trav = tob_frame['hob'].trajectory(t_steps_ob_in_trav)

# Calculate in-bound home journey in traveller frame
v_hib_in_tib = -v_tib_in_hib
tib_frame_trav['hib'] = Point(v_hib_in_tib, traj_hob_in_trav.pos[-1,:], traj_hob_in_trav.time[-1])
traj_hib_in_trav = tib_frame_trav['hib'].trajectory(t_steps_ib_in_trav)

# Set and calculate traveller trajectories in traveller frame
traj_tob_in_trav = tob_frame['tob'].trajectory(t_steps_ob_in_trav)
traj_tib_in_trav = tib_frame_trav['tib'].trajectory(t_steps_ib_in_trav)

## Concatenate Trajectories to form Paths; need to revise this so that all variables are in traveller frame, not tob or tib
path_home_in_trav = traj_hob_in_trav.concatenate(traj_hib_in_trav)
path_trav_in_trav = traj_tob_in_trav.concatenate(traj_tib_in_trav)

t_tob_in_tob = 0
path_trav_in_trav = Path.create_path(s_sep_in_tob, t_tob_in_tob, [t_obj_in_tob, t_ibj_in_tib_trav], [v_zero, v_zero])
t_hob_in_tob = 0
v_hob_in_tob = -v_tob_in_hob
v_hib_in_tib = -v_tib_in_hib
path_home_in_trav = Path.create_path(s_sep_in_tob, t_hob_in_tob, [t_obj_in_tob, t_ibj_in_tib_trav], [v_hob_in_tob, v_hib_in_tib])


### Out-Bound Traveller Plot

v_hib_in_tob = v_sum(v_hib_in_hob_stationary, v_hob_in_tob)
tob_frame['hib'] = Point(v_hib_in_tob, traj_hob_in_trav.pos[-1,:], traj_hob_in_trav.time[-1])
traj_hib_in_tob = tob_frame['hib'].trajectory(t_steps_ib_in_tib_trav)

v_tib_in_hob = v_sum(v_tib_in_hib, v_hib_in_hob_stationary)
v_tib_in_tob = v_sum(v_tib_in_hob, v_hob_in_tob)
tob_frame['tib'] = Point(v_tib_in_tob, traj_tob_in_trav.pos[-1,:], traj_tob_in_trav.time[-1])
traj_tib_in_tob = tob_frame['tib'].trajectory(t_steps_ib_in_tib_trav)

path_home_in_tob = traj_hob_in_trav.concatenate(traj_hib_in_tob)
path_trav_in_tob = traj_tob_in_trav.concatenate(traj_tib_in_tob)


v_home_in_tob = v_hob_in_tob
t_fj_in_tob = dilate_time(t_obj_in_hob + t_ibj_in_hib_stationary, v_home_in_tob, s_sep_in_hib_stationary)
path_home_in_tob = Path.create_path(s_sep_in_tob, t_hob_in_tob, [t_fj_in_tob], [v_home_in_tob])

s_zero = np.array([0,0,0], dtype=np.float32)
# t_obj_in_tob is contracted because it is a function of the time that passes in the home frame, at position 0 in the tob (native, unknown) frame, i.e. it's the presence duration of a distance
# t_ibj_in_tob is dilated because it is a function of the time that passes at position 0 in the tib (foreign, known) frame
t_ibj_in_tob = dilate_time(t_ibj_in_tib_trav, v_tib_in_tob, s_zero)
# TODO: Ultimately, the ib journey time should probably be calculated by taking the distance between tib and hib and dividing by their velocity difference. This way, no assumption is made about the final result before it is calculated. The same thing should be done for the obj in the tib frame, since the tib start values are not defined before the tap until then.
path_trav_in_tob = Path.create_path(s_sep_in_tob, t_tob_in_tob, [t_obj_in_tob, t_ibj_in_tob], [v_zero, v_tib_in_tob])


### In-Bound Traveller Plot

#v_tob_in_tib = -v_tib_in_tob
#v_hob_in_tib = v_sum(v_hob_in_tob, v_tob_in_tib)
#tib_frame['hob'] = Point(v_hob_in_tib, )

v_home_in_tib = v_hib_in_tib
t_fj_in_tib = dilate_time(t_obj_in_hob + t_ibj_in_hib_stationary, v_home_in_tib, s_sep_in_hib_stationary)
v_tob_in_tib = -v_tib_in_tob
# TODO: implement transform function between frames with mismatching origins
#s_sep_in_tib = 
#t_hob_in_tib = 
#path_home_in_tib = Path.create_path(s_sep_in_tib, t_hob_in_tib, [t_fj_in_tib], [v_home_in_tib])

s_zero = np.array([0,0,0], dtype=np.float32)
# See above explanations for why dilation and contraction are both used here
# TODO: probably figure out some function or interface that more intuitively and easily/readily calculates s or t without having to do all this thought and explanation
t_obj_in_tib = dilate_time(t_obj_in_tob, v_tob_in_tib, s_zero)
t_ibj_in_tib = contract_time(t_ibj_in_hib_stationary, v_hib_in_tib, s_zero)
#path_trav_in_tib = Path.create_path(s_sep_in_tib, t_tob_in_tib, [t_obj_in_tib, t_ibj_in_tib], [v_tob_in_tib, v_zero])

# ==== Output ====

# Plot journeys in home frame
fig = plt.figure()
fig.suptitle('Twin Trajectories')

ax_journey = fig.add_subplot(2,2,1)
ax_journey.set_title('Journey in Home Frame')
#plot_spacetime(ax_journey, (path_home_in_home, path_trav_in_home), do_cones=False)
plot_paths(ax_journey, (path_home_in_home, path_trav_in_home))

ax_journey = fig.add_subplot(2,2,2)
ax_journey.set_title('Journey in Traveller Frame')
#plot_spacetime(ax_journey, (path_home_in_trav, path_trav_in_trav), do_cones=False)
plot_paths(ax_journey, (path_trav_in_trav, path_home_in_trav))

ax_journey = fig.add_subplot(2,2,3)
ax_journey.set_title('Journey in Out-Bound Traveller Frame')
#plot_spacetime(ax_journey, (path_home_in_tob, path_trav_in_tob), do_cones=False)
plot_paths(ax_journey, (path_home_in_tob, path_trav_in_tob))

# Plot settings
plt.subplots_adjust(wspace=0, hspace=0.3)
plt.show()
