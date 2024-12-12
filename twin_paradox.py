import numpy as np
from vector_math import *
from relativity import *
from frame import *
from plot_functions import *

# ==== Setup ====

## Manual settings

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
v_tii_in_hii_x = c * -0.5
v_tia_in_hia_x = v_tii_in_hii_x
v_cii_in_hii_x = v_cob_in_hob_x

# In-bound in out-bound
v_hii_in_hob_x = 0 # Inertial: Relate in-bound journey to out-bound journey by declaring v_hib = v_hob relative to home, tob, and tib, but not traveller
v_tia_in_tob_x = 0 # Accelerating with traveller

## Vectorize

v_tii_in_hii = vector(v_tii_in_hii_x, 0, 0)
v_hii_in_hob = vector(v_hii_in_hob_x, 0, 0)
v_tia_in_tob = vector(v_tia_in_tob_x, 0, 0)
v_tia_in_hia = vector(v_tia_in_hia_x, 0, 0)

## Transform

v_hia_in_tia = -v_tia_in_hia
v_hia_in_tob = v_transform(v_hia_in_tia, v_tia_in_tob)
v_tii_in_hob = v_transform(v_tii_in_hii, v_hii_in_hob)

## Set endpoint velocities (just so they have velocities; it doesn't really matter)
v_hei_in_hob = v_hii_in_hob
v_tei_in_hob = v_tii_in_hob
v_hea_in_tob = v_hia_in_tob
v_tea_in_tob = v_tia_in_tob


# ==== Scene ====

### Home Plot: hob frame where velocity of hib in hob = 0
# Note: Using hob frame for out-bound and hib frame for in-bound journey is improper, since in the case where velocity of hib in hob =/= 0, the paths would be discontinuous

## Initialize frame

hob_frame = Frame('hob', pos=[s_hob_in_hob_x,0,0], time=t_hob_in_hob)
hob_frame.add_path('home_inertial', 'hob')
hob_frame.add_point('tob', vel=[v_tob_in_hob_x,0,0], pos=[s_tob_in_hob_x,0,0], time=t_tob_in_hob)
hob_frame.add_path('traveller_inertial', 'tob')
hob_frame.add_point('cob', vel=[v_cob_in_hob_x,0,0], pos=[s_cob_in_hob_x,0,0], time=t_cob_in_hob)

## Calculate points in journey

# Out-bound: start to changepoint
t_diff_ib_ob_in_hob = hob_frame['tob'].convergence_time_with(hob_frame['cob'])
hob_frame.inertial_step('hob', 'hii', t_diff_ib_ob_in_hob, v_hii_in_hob) # stationary hib
hob_frame.paths['home_inertial'].add_segment(hob_frame['hob'], hob_frame['hii'])
hob_frame.inertial_step('tob', 'tii', t_diff_ib_ob_in_hob, v_tii_in_hob)
hob_frame.paths['traveller_inertial'].add_segment(hob_frame['tob'], hob_frame['tii'])

# In-bound: changepoint to end
t_diff_ep_ib_in_hob_i = hob_frame['tii'].convergence_time_with(hob_frame['hii'])
hob_frame.inertial_step('hii', 'hei', t_diff_ep_ib_in_hob_i, v_hei_in_hob)
hob_frame.paths['home_inertial'].add_segment(hob_frame['hii'], hob_frame['hei'])
hob_frame.inertial_step('tii', 'tei', t_diff_ep_ib_in_hob_i, v_tei_in_hob)
hob_frame.paths['traveller_inertial'].add_segment(hob_frame['tii'], hob_frame['tei'])

# Endpoint
hob_frame.paths['home_inertial'].add_point(hob_frame['hei'].pos, hob_frame['hei'].time)
hob_frame.paths['traveller_inertial'].add_point(hob_frame['tei'].pos, hob_frame['tei'].time)


### Traveller Plot

## Initialize frame

v_hob_in_tob = -hob_frame['tob'].vel
tob_in_tob = hob_frame['tob'].translate_full('pos', v_hob_in_tob)
tob_frame = Frame('tob', pos=tob_in_tob.pos, time=tob_in_tob.time)
tob_frame.add_path('traveller_accelerating', 'tob')
hob_in_tob = hob_frame['hob'].translate_full('pos', v_hob_in_tob)
tob_frame.add_point('hob', vel=hob_in_tob.vel, pos=hob_in_tob.pos, time=hob_in_tob.time)
tob_frame.add_path('home_accelerating', 'hob')
cob_in_tob = hob_frame['cob'].translate_full('pos', v_hob_in_tob)
tob_frame.add_point('cob', vel=cob_in_tob.vel, pos=cob_in_tob.pos, time=cob_in_tob.time)

## Calculate points in journey

# Out-bound: start to changepoint
t_diff_ib_ob_in_tob = tob_frame['tob'].convergence_time_with(tob_frame['cob'])
tob_frame.inertial_step('tob', 'tia', t_diff_ib_ob_in_tob, v_tia_in_tob)
tob_frame.paths['traveller_accelerating'].add_segment(tob_frame['tob'], tob_frame['tia'])
tob_frame.inertial_step('hob', 'hia', t_diff_ib_ob_in_tob, v_hia_in_tob)
tob_frame.paths['home_accelerating'].add_segment(tob_frame['hob'], tob_frame['hia'])

# In-bound: changepoint to end
t_diff_ea_ib_in_tob = tob_frame['tia'].convergence_time_with(tob_frame['hia']) # t_diff uses ib instead of ia because the event ib = ia, while objects [h/t]ia have velocity dependent on i/a
tob_frame.inertial_step('tia', 'tea', t_diff_ea_ib_in_tob, v_tea_in_tob)
tob_frame.paths['traveller_accelerating'].add_segment(tob_frame['tia'], tob_frame['tea'])
tob_frame.inertial_step('hia', 'hea', t_diff_ea_ib_in_tob, v_hea_in_tob)
tob_frame.paths['home_accelerating'].add_segment(tob_frame['hia'], tob_frame['hea'])

# Endpoint
tob_frame.paths['traveller_accelerating'].add_point(tob_frame['tea'].pos, tob_frame['tea'].time)
tob_frame.paths['home_accelerating'].add_point(tob_frame['hea'].pos, tob_frame['hea'].time)

# Temp
temp_path_trav_trav = tob_frame.paths['traveller_accelerating']
temp_path_home_trav = tob_frame.paths['home_accelerating']


## Old Calculations

hib_in_hib_i = hob_frame['hii'].transform(hob_frame['hii'].vel)
hib_frame_i = Frame('hii', hib_in_hib_i.pos, hib_in_hib_i.time) # stationary hib frame
v_hob_in_hib_i = -v_hii_in_hob
hib_frame_i['tii'] = hob_frame['tii'].transform(v_hob_in_hib_i)
hib_frame_i['hii'] = hob_frame['hii'].transform(v_hob_in_hib_i)
hib_frame_i['hei'] = hob_frame['hei'].transform(v_hob_in_hib_i)

v_tib_in_tob_trav = np.array([0,0,0], dtype=np.float32)
v_tob_in_tib_trav = -v_tib_in_tob_trav
# Not sure if these are needed
v_tib_in_hob_trav = v_transform(v_tib_in_tob_trav, hob_frame['tob'].vel)
v_hia_in_tia = -hib_frame_i['tii'].vel
v_hib_in_hob_trav = np.array(v_transform(v_hia_in_tia, v_tib_in_hob_trav), dtype=np.float32)
v_hob_in_hib_trav = -v_hib_in_hob_trav

# Out-Bound Journey

# Calculate pos and time of traveller's turnaround point; pos is relative to traveller at the beginning of its journey
v_hob_in_tob = -hob_frame['tob'].vel
s_hob_in_hob = np.array([s_hob_in_hob_x,0,0], dtype=np.float32)
t_obj_in_tob = t_transform_down(t_diff_ib_ob_in_hob, v_hob_in_tob, s_hob_in_hob)
#t_obj_in_tob = (s_tap_in_tob[0] - s_sep_in_tob[0]) / -v_hob_in_tob[0]

# Set out-bound traveller in its own frame
tob_frame = Frame('tob')

# In-Bound Journey

s_cob_in_hob = np.array([s_cob_in_hob_x,0,0], dtype=np.float32)
s_tap_in_tob = s_transform_down(s_cob_in_hob, v_hob_in_tob, s_hob_in_hob)
s_tap_in_tib_trav = s_transform_down(s_tap_in_tob, v_tob_in_tib_trav, tob_frame['tob'].time) # TODO: in tib, tap is 0 and sep is negative
s_sep_in_tob = s_hob_in_hob # TODO: use v_tob_in_hob
s_sep_in_tib_trav = s_transform_down(s_sep_in_tob, v_tob_in_tib_trav, tob_frame['tob'].time)
t_ibj_in_tib_trav = (s_sep_in_tib_trav[0] - s_tap_in_tib_trav[0]) / v_tii_in_hii_x

# Set in-bound traveller in its own frame
t_tib_in_tib_trav = t_transform_down(t_obj_in_tob, v_tob_in_tib_trav, tob_frame['tob'].pos)
tib_frame_trav = Frame('tib', time=t_tib_in_tib_trav)

# Trajectories

# Calculate time steps for trajectory calculations
t_steps_ob_in_tob = np.linspace(0, t_obj_in_tob, 100)
t_steps_ib_in_tib_trav = np.linspace(0, t_ibj_in_tib_trav, 100)
t_steps_ob_in_trav = t_steps_ob_in_tob
t_steps_ib_in_trav = t_steps_ib_in_tib_trav

# Calculate out-bound journey of home in traveller frame
s_hob_in_tob = s_transform_down(hob_frame['hob'].pos, v_hob_in_tob, hob_frame['hob'].time)
t_hob_in_tob = t_transform_down(hob_frame['hob'].time, v_hob_in_tob, hob_frame['hob'].pos)
#tob_frame['hob'] = Point(v_hob_in_tob, s_hob_in_tob, t_hob_in_tob)
tob_frame['hob'] = hob_frame['hob'].transform(v_hob_in_tob)
#tob_frame['hob'] = hob_frame['hob'].transform(v_hob_in_tob)

# Calculate out-bound trajectory in traveller frame
traj_hob_in_trav = tob_frame['hob'].trajectory(t_steps_ob_in_trav)

# Calculate in-bound home journey in traveller frame
v_hia_in_tia = -hib_frame_i['tii'].vel
tib_frame_trav['hib'] = Point(v_hia_in_tia, traj_hob_in_trav.pos[-1,:], traj_hob_in_trav.time[-1])
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
v_hob_in_tob = -hob_frame['tob'].vel
v_hia_in_tia = -hib_frame_i['tii'].vel
path_home_in_trav = Path.create_path(s_sep_in_tob, t_hob_in_tob, [t_obj_in_tob, t_ibj_in_tib_trav], [v_hob_in_tob, v_hia_in_tia])


### Out-Bound Traveller Plot

## Initialize paths
tob_frame.add_path('traveller_inertial', 'tob')
tob_frame.add_path('home_inertial', 'hob')

## Calculate points in journey

# Out-bound: start to changepoint
v_tii_in_tob = v_transform(v_tii_in_hob, v_hob_in_tob)
tob_frame.inertial_step('tob', 'tii', t_diff_ib_ob_in_tob, v_tii_in_tob)
tob_frame.paths['traveller_inertial'].add_segment(tob_frame['tob'], tob_frame['tii'])
v_hii_in_tob = v_transform(v_hii_in_hob, v_hob_in_tob)
tob_frame.inertial_step('hob', 'hii', t_diff_ib_ob_in_tob, v_hii_in_tob)
tob_frame.paths['home_inertial'].add_segment(tob_frame['hob'], tob_frame['hii'])

# In-bound: changepoint to end
t_diff_ei_ib_in_tob = tob_frame['tii'].convergence_time_with(tob_frame['hii'])
v_tei_in_tob = v_transform(v_tei_in_hob, v_hob_in_tob)
tob_frame.inertial_step('tii', 'tei', t_diff_ei_ib_in_tob, v_tei_in_tob)
tob_frame.paths['traveller_inertial'].add_segment(tob_frame['tii'], tob_frame['tei'])
v_hei_in_tob = v_transform(v_hei_in_hob, v_hob_in_tob)
tob_frame.inertial_step('hii', 'hei', t_diff_ei_ib_in_tob, v_hei_in_tob)
tob_frame.paths['home_inertial'].add_segment(tob_frame['hii'], tob_frame['hei'])

# Endpoint
tob_frame.paths['traveller_inertial'].add_point(tob_frame['tei'].pos, tob_frame['tei'].time)
tob_frame.paths['home_inertial'].add_point(tob_frame['hei'].pos, tob_frame['hei'].time)

# Temp
temp_path_trav_tob = tob_frame.paths['traveller_inertial']
temp_path_home_tob = tob_frame.paths['home_inertial']


## Old Calculations

v_hib_in_tob = v_transform(v_hii_in_hob, v_hob_in_tob)
tob_frame['hib'] = Point(v_hib_in_tob, traj_hob_in_trav.pos[-1,:], traj_hob_in_trav.time[-1])
traj_hib_in_tob = tob_frame['hib'].trajectory(t_steps_ib_in_tib_trav)

v_tib_in_hob = v_transform(hib_frame_i['tii'].vel, v_hii_in_hob)
v_tib_in_tob = v_transform(v_tib_in_hob, v_hob_in_tob)
tob_frame['tib'] = Point(v_tib_in_tob, traj_tob_in_trav.pos[-1,:], traj_tob_in_trav.time[-1])
traj_tib_in_tob = tob_frame['tib'].trajectory(t_steps_ib_in_tib_trav)

path_home_in_tob = traj_hob_in_trav.concatenate(traj_hib_in_tob)
path_trav_in_tob = traj_tob_in_trav.concatenate(traj_tib_in_tob)


v_home_in_tob = v_hob_in_tob
t_fj_in_tob = t_transform(t_diff_ib_ob_in_hob + t_diff_ep_ib_in_hob_i, v_home_in_tob, hib_frame_i['hei'].pos)
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

## Initialize paths
v_hob_in_tii = -hob_frame['tii'].vel
tii_in_tii = hob_frame['tii'].translate_full('pos', v_hob_in_tii)
tii_frame = Frame('tii', pos=tii_in_tii.pos, time=tii_in_tii.time) # TODO: I don't think tii_in_tii.time is right?
tob_in_tii = hob_frame['tob'].translate_full('pos', v_hob_in_tii) # TODO: this function assumes equivalent origins, which is not the case, and so tob incorrectly starts at pos 0
tii_frame.add_point('tob', vel=tob_in_tii.vel, pos=tob_in_tii.pos, time=tob_in_tii.time)
tii_frame.add_path('traveller_inertial', 'tob')
hob_in_tii = hob_frame['hob'].translate_full('pos', v_hob_in_tii)
tii_frame.add_point('hob', vel=hob_in_tii.vel, pos=hob_in_tii.pos, time=hob_in_tii.time)
tii_frame.add_path('home_inertial', 'hob')
cob_in_tii = hob_frame['cob'].translate_full('pos', v_hob_in_tii)
tii_frame.add_point('cob', vel=cob_in_tii.vel, pos=cob_in_tii.pos, time=cob_in_tii.time)

## Calculate points in journey

# Out-bound: start to changepoint
t_diff_ib_ob_in_tii = tii_frame['tob'].convergence_time_with(tii_frame['cob'])
v_tii_in_tii = v_transform(v_tii_in_hob, v_hob_in_tii)
tii_frame.inertial_step('tob', 'tii', t_diff_ib_ob_in_tii, v_tii_in_tii)
tii_frame.paths['traveller_inertial'].add_segment(tii_frame['tob'], tii_frame['tii'])
v_hii_in_tii = v_transform(v_hii_in_hob, v_hob_in_tii)
tii_frame.inertial_step('hob', 'hii', t_diff_ib_ob_in_tii, v_hii_in_tii)
tii_frame.paths['home_inertial'].add_segment(tii_frame['hob'], tii_frame['hii'])

# In-bound: changepoint to end
t_diff_ei_ib_in_tii = tii_frame['tii'].convergence_time_with(tii_frame['hii'])
v_tei_in_tii = v_transform(v_tei_in_hob, v_hob_in_tii)
tii_frame.inertial_step('tii', 'tei', t_diff_ei_ib_in_tii, v_tei_in_tii)
tii_frame.paths['traveller_inertial'].add_segment(tii_frame['tii'], tii_frame['tei'])
v_hei_in_tii = v_transform(v_hei_in_hob, v_hob_in_tii)
tii_frame.inertial_step('hii', 'hei', t_diff_ei_ib_in_tii, v_hei_in_tii)
tii_frame.paths['home_inertial'].add_segment(tii_frame['hii'], tii_frame['hei'])

# Endpoint
tii_frame.paths['traveller_inertial'].add_point(tii_frame['tei'].pos, tii_frame['tei'].time)
tii_frame.paths['home_inertial'].add_point(tii_frame['hei'].pos, tii_frame['hei'].time)

# Temp
temp_path_trav_tii = tii_frame.paths['traveller_inertial']
temp_path_home_tii = tii_frame.paths['home_inertial']


## Old Calculations

#v_tob_in_tib = -v_tib_in_tob
#v_hob_in_tib = v_sum(v_hob_in_tob, v_tob_in_tib)
#tib_frame['hob'] = Point(v_hob_in_tib, )

v_home_in_tib = v_hia_in_tia
t_fj_in_tib = t_transform(t_diff_ib_ob_in_hob + t_diff_ep_ib_in_hob_i, v_home_in_tib, hib_frame_i['hei'].pos)
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
t_ibj_in_tib = t_transform_down(t_diff_ep_ib_in_hob_i, v_hia_in_tia, s_zero)
t_tob_in_tib = 0
path_trav_in_tib = Path.create_path(s_sep_in_tib, t_tob_in_tib, [t_obj_in_tib, t_ibj_in_tib], [v_tob_in_tib, v_zero])

# ==== Output ====

# Plot journeys in home frame
fig = plt.figure()
fig.suptitle('Twin Trajectories')

ax_journey = fig.add_subplot(2,2,1)
ax_journey.set_title('Journey in Inertial Home Frame')
#plot_spacetime(ax_journey, (hob_frame.paths['home_inertial'], hob_frame.paths['traveller_inertial']), do_cones=False)
plot_paths(ax_journey, (hob_frame.paths['home_inertial'], hob_frame.paths['traveller_inertial']))

ax_journey = fig.add_subplot(2,2,2)
ax_journey.set_title('Journey in Accelerating Traveller Frame')
#plot_spacetime(ax_journey, (path_home_in_trav, path_trav_in_trav), do_cones=False)
plot_paths(ax_journey, (temp_path_trav_trav, temp_path_home_trav)) #(path_trav_in_trav, path_home_in_trav))

ax_journey = fig.add_subplot(2,2,3)
ax_journey.set_title('Journey in Inertial Out-Bound Traveller Frame')
#plot_spacetime(ax_journey, (path_home_in_tob, path_trav_in_tob), do_cones=False)
plot_paths(ax_journey, (temp_path_home_tob, temp_path_trav_tob))

ax_journey = fig.add_subplot(2,2,4)
ax_journey.set_title('Journey in Inertial In-Bound Traveller Frame')
plot_paths(ax_journey, (temp_path_home_tii, temp_path_trav_tii)) #(path_home_in_tib, path_trav_in_tib))

# Plot settings
plt.subplots_adjust(wspace=0, hspace=0.3)
plt.show()
