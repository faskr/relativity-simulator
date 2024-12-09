import numpy as np
from vector_math import *
from relativity_math import *
from same_frame_math import *
from plot_functions import *

# ==== Setup ====

# Manual settings

# Home frame during out-bound journey
v_cob_in_hob_x = 0 # "change" or turn-around point
v_tob_in_hob_x = c * 0.5 # traveller

s_hob_in_hob_x = 0 # Start and end point in out-bound home frame
s_cob_in_hob_x = 50 # Turn-around point in out-bound home frame
s_tob_in_hob_x = s_hob_in_hob_x

t_hob_in_hob = 0
t_cob_in_hob = t_hob_in_hob
t_tob_in_hob = t_hob_in_hob

# Home frame during in-bound journey
v_tib_in_hib_x = c * -0.5
v_cib_in_hib_x = v_cob_in_hob_x

# ==== Scene ====

### Home Plot

## Out-bound journey

hob_frame = Frame('hob', pos=[s_hob_in_hob_x,0,0], time=t_hob_in_hob)
hob_frame['cob'] = Point(vel=[v_cob_in_hob_x,0,0], pos=[s_cob_in_hob_x,0,0], time=t_cob_in_hob)
hob_frame['tob'] = Point(vel=[v_tob_in_hob_x,0,0], pos=[s_tob_in_hob_x,0,0], time=t_tob_in_hob)

## In-bound journey

t_diff_ib_ob_in_hob = convergence_time(hob_frame['cob'], hob_frame['tob'])

# TODO: abstract these (v), s, and t calculations into a waypoint function (calculates Events?) in same_frame_math.py
# Velocity difference between out-bound and in-bound sections of home frame
v_hib_in_hob_s = np.array([0,0,0], dtype=np.float32) # Relate in-bound journey to out-bound journey by declaring v_hib = v_hob relative to home, tob, and tib, but not traveller
s_hib_in_hob = hob_frame['hob'].pos + hob_frame['hob'].vel * t_diff_ib_ob_in_hob
t_hib_in_hob = hob_frame['hob'].time + t_diff_ib_ob_in_hob
hob_frame['hib_s'] = Point(vel=v_hib_in_hob_s, pos=s_hib_in_hob, time=t_hib_in_hob) # stationary hib

# Could also calc hib_frame['tob'] using contract_to_motion, to then get s and t of tib in hib, and use v_tib_in_hib, to then get hib_frame['tib']
# Not sure which way I prefer yet
v_tib_in_hob_x_s = v_sum(v_tib_in_hib_x, v_hib_in_hob_s[0])
s_tib_in_hob = hob_frame['tob'].pos + hob_frame['tob'].vel * t_diff_ib_ob_in_hob
t_tib_in_hob = hob_frame['tob'].time + t_diff_ib_ob_in_hob
hob_frame['tib_s'] = Point(vel=[v_tib_in_hob_x_s,0,0], pos=s_tib_in_hob, time=t_tib_in_hob)

v_cib_in_hob_x_s = v_sum(v_cib_in_hib_x, v_hib_in_hob_s[0])
s_cib_in_hob = hob_frame['cob'].pos + hob_frame['cob'].vel * t_diff_ib_ob_in_hob
t_cib_in_hob = hob_frame['cob'].time + t_diff_ib_ob_in_hob
hob_frame['cib_s'] = Point(vel=[v_cib_in_hob_x_s,0,0], pos=s_cib_in_hob, time=t_cib_in_hob)

## End-point

t_diff_ep_ib_in_hob_s = convergence_time(hob_frame['tib_s'], hob_frame['hib_s'])

# Have to calculate hib in hob first before in its own frame, because s_sep and therefore s_hib is given in hob
s_hep_in_hob_s = hob_frame['hib_s'].pos + hob_frame['hib_s'].vel * t_diff_ep_ib_in_hob_s
t_hep_in_hob_s = hob_frame['hib_s'].time + t_diff_ep_ib_in_hob_s
v_hep_in_hob_s = v_hib_in_hob_s # lazy assertion, but it doesn't really matter
hob_frame['hep_s'] = Point(vel=v_hep_in_hob_s, pos=s_hep_in_hob_s, time=t_hep_in_hob_s)

## Transform to in-bound frame

hib_in_hib_s = hob_frame['hib_s'].new_frame(hob_frame['hib_s'].vel)
hib_frame_s = Frame('hib_s', hib_in_hib_s.pos, hib_in_hib_s.time) # stationary hib frame
v_hob_in_hib_s = -v_hib_in_hob_s
hib_frame_s['tib_s'] = hob_frame['tib_s'].new_frame(v_hob_in_hib_s)
hib_frame_s['hib_s'] = hob_frame['hib_s'].new_frame(v_hob_in_hib_s)
hib_frame_s['hep_s'] = hob_frame['hep_s'].new_frame(v_hob_in_hib_s)

## Junk for reference
s_cob_in_hob = np.array([s_cob_in_hob_x,0,0], dtype=np.float32)
s_hob_in_hob = np.array([s_hob_in_hob_x,0,0], dtype=np.float32)
# Contract (supposing v_hob_in_hib =/= 0):
#   s_tap and s_sep are defined in hob, but measured simultaneously in hib, i.e. s_sep and s_tap are lengths
#   t_ibj in hob is defined in hob, but is then measured at a point in space that is the same in hib throughout the measurement, i.e. t_ibj is a presence duration

## Paths

t_diff_ep_ib_in_hib_s = convergence_time(hib_frame_s['tib_s'], hib_frame_s['hib_s'])

path_home_in_home = Path.create_path(hob_frame['hob'].pos, hob_frame['hob'].time, [t_diff_ib_ob_in_hob, t_diff_ep_ib_in_hib_s], [hob_frame['hob'].vel, hib_frame_s['hib_s'].vel])
t_tob_in_hob = 0
v_tob_in_hob = np.array([v_tob_in_hob_x,0,0], dtype=np.float32)
v_tib_in_hib = np.array([v_tib_in_hib_x,0,0], dtype=np.float32)
path_trav_in_home = Path.create_path(s_hob_in_hob, t_tob_in_hob, [t_diff_ib_ob_in_hob, t_diff_ep_ib_in_hib_s], [v_tob_in_hob, v_tib_in_hib])

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
t_obj_in_tob = contract_time(t_diff_ib_ob_in_hob, v_hob_in_tob, s_hob_in_hob)
#t_obj_in_tob = (s_tap_in_tob[0] - s_sep_in_tob[0]) / -v_hob_in_tob[0]

# Set out-bound traveller in its own frame
tob_frame = Frame('tob')

# In-Bound Journey

s_tap_in_tob = contract_space(s_cob_in_hob, v_hob_in_tob, s_hob_in_hob)
s_tap_in_tib_trav = contract_space(s_tap_in_tob, v_tob_in_tib_trav, tob_frame['tob'].time) # TODO: in tib, tap is 0 and sep is negative
s_sep_in_tob = s_hob_in_hob # TODO: use v_tob_in_hob
s_sep_in_tib_trav = contract_space(s_sep_in_tob, v_tob_in_tib_trav, tob_frame['tob'].time)
t_ibj_in_tib_trav = (s_sep_in_tib_trav[0] - s_tap_in_tib_trav[0]) / v_tib_in_hib_x

# Set in-bound traveller in its own frame
t_tib_in_tib_trav = contract_time(t_obj_in_tob, v_tob_in_tib_trav, tob_frame['tob'].pos)
tib_frame_trav = Frame('tib', time=t_tib_in_tib_trav)

# Trajectories

# Calculate time steps for trajectory calculations
t_steps_ob_in_tob = np.linspace(0, t_obj_in_tob, 100)
t_steps_ib_in_tib_trav = np.linspace(0, t_ibj_in_tib_trav, 100)
t_steps_ob_in_trav = t_steps_ob_in_tob
t_steps_ib_in_trav = t_steps_ib_in_tib_trav

# Calculate out-bound journey of home in traveller frame
s_hob_in_tob = contract_space(hob_frame['hob'].pos, v_hob_in_tob, hob_frame['hob'].time)
t_hob_in_tob = contract_time(hob_frame['hob'].time, v_hob_in_tob, hob_frame['hob'].pos)
#tob_frame['hob'] = Point(v_hob_in_tob, s_hob_in_tob, t_hob_in_tob)
tob_frame['hob'] = hob_frame['hob'].new_frame(v_hob_in_tob)
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
v_zero = np.array([0,0,0], dtype=np.float32)
path_trav_in_trav = Path.create_path(s_sep_in_tob, t_tob_in_tob, [t_obj_in_tob, t_ibj_in_tib_trav], [v_zero, v_zero])
t_hob_in_tob = 0
v_hob_in_tob = -v_tob_in_hob
v_hib_in_tib = -v_tib_in_hib
path_home_in_trav = Path.create_path(s_sep_in_tob, t_hob_in_tob, [t_obj_in_tob, t_ibj_in_tib_trav], [v_hob_in_tob, v_hib_in_tib])


### Out-Bound Traveller Plot

v_hib_in_tob = v_sum(v_hib_in_hob_s, v_hob_in_tob)
tob_frame['hib'] = Point(v_hib_in_tob, traj_hob_in_trav.pos[-1,:], traj_hob_in_trav.time[-1])
traj_hib_in_tob = tob_frame['hib'].trajectory(t_steps_ib_in_tib_trav)

v_tib_in_hob = v_sum(v_tib_in_hib, v_hib_in_hob_s)
v_tib_in_tob = v_sum(v_tib_in_hob, v_hob_in_tob)
tob_frame['tib'] = Point(v_tib_in_tob, traj_tob_in_trav.pos[-1,:], traj_tob_in_trav.time[-1])
traj_tib_in_tob = tob_frame['tib'].trajectory(t_steps_ib_in_tib_trav)

path_home_in_tob = traj_hob_in_trav.concatenate(traj_hib_in_tob)
path_trav_in_tob = traj_tob_in_trav.concatenate(traj_tib_in_tob)


v_home_in_tob = v_hob_in_tob
t_fj_in_tob = transform_time(t_diff_ib_ob_in_hob + t_diff_ep_ib_in_hib_s, v_home_in_tob, hib_frame_s['hep_s'].pos)
path_home_in_tob = Path.create_path(s_sep_in_tob, t_hob_in_tob, [t_fj_in_tob], [v_home_in_tob])

# t_obj_in_tob is contracted because it is a function of the time that passes in the home frame, at position 0 in the tob (native, unknown) frame, i.e. it's the presence duration of a distance
# t_ibj_in_tob is dilated because it is a function of the time that passes at position 0 in the tib (foreign, known) frame
#s_zero = np.array([0,0,0], dtype=np.float32)
#t_ibj_in_tob = transform_time(t_ibj_in_tib_trav, v_tib_in_tob, s_zero)

# Ultimately, the ib journey time should probably be calculated by taking the distance between tib and hib and dividing by their velocity difference. This way, no assumption is made about the final result before it is calculated. The same thing should be done for the obj in the tib frame, since the tib start values are not defined before the tap until then.
v_diff_tib_hib_in_tob = v_tib_in_tob - v_hib_in_tob
s_diff_hib_tib_in_tob = tob_frame['hib'].pos - tob_frame['tib'].pos
t_ibj_in_tob = proj(s_diff_hib_tib_in_tob, v_diff_tib_hib_in_tob) / mag(v_diff_tib_hib_in_tob)
path_trav_in_tob = Path.create_path(s_sep_in_tob, t_tob_in_tob, [t_obj_in_tob, t_ibj_in_tob], [v_zero, v_tib_in_tob])


### In-Bound Traveller Plot

#v_tob_in_tib = -v_tib_in_tob
#v_hob_in_tib = v_sum(v_hob_in_tob, v_tob_in_tib)
#tib_frame['hob'] = Point(v_hob_in_tib, )

v_home_in_tib = v_hib_in_tib
t_fj_in_tib = transform_time(t_diff_ib_ob_in_hob + t_diff_ep_ib_in_hib_s, v_home_in_tib, hib_frame_s['hep_s'].pos)
v_tob_in_tib = -v_tib_in_tob
# TODO: implement transform function between frames with mismatching origins
#s_sep_in_tib = 
#t_hob_in_tib = 
v_hob_in_tib = -v_tib_in_hob
v_diff_tob_hob_in_tib = v_tob_in_tib - v_hob_in_tib
s_tib_in_tib = np.array([0,0,0], dtype=np.float32)
s_hib_in_tib = -s_tap_in_tob
s_diff_tib_hib_in_tib = s_tib_in_tib - s_hib_in_tib
t_obj_in_tib = proj(s_diff_tib_hib_in_tib, v_diff_tob_hob_in_tib) / mag(v_diff_tob_hob_in_tib)
s_zero = np.array([0,0,0], dtype=np.float32)

s_sep_in_tib = v_tob_in_tib * -t_obj_in_tib
t_hob_in_tib = 0
path_home_in_tib = Path.create_path(s_sep_in_tib, t_hob_in_tib, [t_fj_in_tib], [v_home_in_tib])


# See above explanations for why dilation and contraction are both used here
# TODO: probably figure out some function or interface that more intuitively and easily/readily calculates s or t without having to do all this thought and explanation
#t_obj_in_tib = transform_time(t_obj_in_tob, v_tob_in_tib, s_zero)
t_ibj_in_tib = contract_time(t_diff_ep_ib_in_hib_s, v_hib_in_tib, s_zero)
t_tob_in_tib = 0
path_trav_in_tib = Path.create_path(s_sep_in_tib, t_tob_in_tib, [t_obj_in_tib, t_ibj_in_tib], [v_tob_in_tib, v_zero])

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

ax_journey = fig.add_subplot(2,2,4)
ax_journey.set_title('Journey in In-Bound Traveller Frame')
plot_paths(ax_journey, (path_home_in_tib, path_trav_in_tib))

# Plot settings
plt.subplots_adjust(wspace=0, hspace=0.3)
plt.show()
