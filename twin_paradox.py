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

hob_frame = Frame('hob')
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


# TODO: It would be nice to have a function that takes all paths in a given frame and transforms them to another frame, so that most of this code below wouldn't be necessary, and inertial steps only need to be done in one frame

### Traveller Plot

## Initialize frame

v_hob_in_tob = -hob_frame['tob'].vel
#tob_in_tob = hob_frame['tob'].translate_full('pos', hob_frame['tob'])
tob_frame = Frame('tob')#, pos=tob_in_tob.pos, time=tob_in_tob.time)
tob_frame.add_path('traveller_accelerating', 'tob')
hob_in_tob = hob_frame['hob'].transform(hob_frame['tob'])
tob_frame.add_point('hob', vel=hob_in_tob.vel, pos=hob_in_tob.pos, time=hob_in_tob.time)
tob_frame.add_path('home_accelerating', 'hob')
cob_in_tob = hob_frame['cob'].transform(hob_frame['tob'])
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


### In-Bound Traveller Plot

## Initialize paths
v_hob_in_tii = -hob_frame['tii'].vel
#tii_in_tii = hob_frame['tii'].translate_full('pos', hob_frame['tii'])
tii_frame = Frame('tii')#, pos=tii_in_tii.pos, time=tii_in_tii.time) # TODO: I don't think tii_in_tii.time is right?
tob_in_tii = hob_frame['tob'].transform(hob_frame['tii'])
tii_frame.add_point('tob', vel=tob_in_tii.vel, pos=tob_in_tii.pos, time=tob_in_tii.time)
tii_frame.add_path('traveller_inertial', 'tob')
hob_in_tii = hob_frame['hob'].transform(hob_frame['tii'])
tii_frame.add_point('hob', vel=hob_in_tii.vel, pos=hob_in_tii.pos, time=hob_in_tii.time)
tii_frame.add_path('home_inertial', 'hob')
cob_in_tii = hob_frame['cob'].transform(hob_frame['tii'])
tii_frame.add_point('cob', vel=cob_in_tii.vel, pos=cob_in_tii.pos, time=cob_in_tii.time)

## Calculate points in journey

# Out-bound: start to changepoint
t_diff_ib_ob_in_tii = tii_frame['tob'].convergence_time_with(tii_frame['cob'])
tii_frame.inertial_step('tob', 'tii', t_diff_ib_ob_in_tii, tii_frame['tii'].vel)
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
plot_paths(ax_journey, (tob_frame.paths['traveller_accelerating'], tob_frame.paths['home_accelerating'])) #(path_trav_in_trav, path_home_in_trav))

ax_journey = fig.add_subplot(2,2,3)
ax_journey.set_title('Journey in Inertial Out-Bound Traveller Frame')
#plot_spacetime(ax_journey, (path_home_in_tob, path_trav_in_tob), do_cones=False)
plot_paths(ax_journey, (tob_frame.paths['traveller_inertial'], tob_frame.paths['home_inertial']))

ax_journey = fig.add_subplot(2,2,4)
ax_journey.set_title('Journey in Inertial In-Bound Traveller Frame')
plot_paths(ax_journey, (tii_frame.paths['traveller_inertial'], tii_frame.paths['home_inertial'])) #(path_home_in_tib, path_trav_in_tib))

# Plot settings
plt.subplots_adjust(wspace=0, hspace=0.3)
plt.show()



### Testing

s_tii_in_hii = hob_frame['tii'].pos # TODO: assertion
t_tii_in_hii = hob_frame['tii'].time - hob_frame['hii'].time # TODO: assertion
tii_in_hii = Point(v_tii_in_hii, s_tii_in_hii, t_tii_in_hii)
#hii_in_tii = hob_frame['hii'].translate_full('pos', tii_in_hii)
hii_offset_by_tii_in_hob = Point(
    hob_frame['hii'].vel,
    hob_frame['hii'].pos - hob_frame['tii'].pos, # hii pos relative to tii pos, i.e. align origins
    hob_frame['hii'].time - hob_frame['tii'].time # hii time relative to tii time, ditto
)
v_hob_in_tii = -hob_frame['tii'].vel
hii_in_tii = hii_offset_by_tii_in_hob.transform(hob_frame['tii'])


# Test translate_full
print('Testing translation')
tii_frame = Frame('tii')
home_in_tii_test1 = hob_frame['hob'].translate_full('pos', hob_frame['tii'])
print(home_in_tii_test1.vel, home_in_tii_test1.pos, home_in_tii_test1.time) # should be 50, -43.3, 0
home_in_tii_test2 = hob_frame['hii'].translate_full('pos', tii_in_hii)
print(home_in_tii_test2.vel, home_in_tii_test2.pos, home_in_tii_test2.time) # should be 50, 6.7, 1

home_in_tii_test3 = hob_frame['hii'].translate_full('pos', hob_frame['tii'], hob_frame['hob'])
print(home_in_tii_test3.vel, home_in_tii_test3.pos, home_in_tii_test3.time) # should be 50, 6.7, 1

change_in_tii_test1 = hob_frame['cob'].translate_full('pos', hob_frame['tii'])
print(change_in_tii_test1.vel, change_in_tii_test1.pos, change_in_tii_test1.time) # should be 50, -43.3, 0 (actually hob)
change_in_tii_test2 = hob_frame['cob'].translate_full('pos', hob_frame['tii'], hob_frame['hob'])
print(change_in_tii_test2.vel, change_in_tii_test2.pos, change_in_tii_test2.time) # should be 50, 0, 0
hob_frame.inertial_step('cob', 'cii', t_diff_ib_ob_in_hob, v_hii_in_hob)
change_in_tii_test3 = hob_frame['cii'].translate_full('pos', tii_in_hii, hob_frame['hii'])
print(change_in_tii_test3.vel, change_in_tii_test3.pos, change_in_tii_test3.time) # should be 50, 50, 1
travob_in_tii_test = hob_frame['tob'].translate_full('pos', tob_frame['tii'])
print(travob_in_tii_test.vel, travob_in_tii_test.pos, travob_in_tii_test.time) # should be 80, 0, 0
travib_in_tii_test = hob_frame['tii'].translate_full('pos', tii_frame['tii'])
print(travib_in_tii_test.vel, travib_in_tii_test.pos, travib_in_tii_test.time) # should be 0, 0, 1

v_tob_in_tii = -v_tii_in_tob
tii_frame.inertial_step_back('tii', 'tob', -t_diff_ib_ob_in_tii, v_tob_in_tii)
travib_in_tob_test = hob_frame['tii'].translate_full('pos', tii_frame['tob'])
print(travib_in_tob_test.vel, travib_in_tob_test.pos, travib_in_tob_test.time) # should be -80, -10.7, 1

tii_frame.add_point('hob', [50,0,0], tii_frame['tob'].pos, tii_frame['tob'].time)
change_in_tob_test = cob_in_tii.translate_full('pos', hob_frame['tob'], tii_frame['hob'])
print(change_in_tob_test.vel, change_in_tob_test.pos, change_in_tob_test.time) # should be -50, 101 (=43.3+57.7), -1.15

travib_in_cob_test = tii_in_hii.translate_full('pos', cob_in_tii)
print(travib_in_cob_test.vel, travib_in_cob_test.pos, travib_in_cob_test.time) # should be -50, 50, 0

hob_in_hob_test = home_in_tii_test1.translate_full('pos', hob_frame['hob'])
print(hob_in_hob_test.vel, hob_in_hob_test.pos, hob_in_hob_test.time) # 0, 0, 0
hob_in_hii = hob_frame['hob'].transform(hob_frame['hii'])
hii_in_hob_test = home_in_tii_test2.translate_full('pos', hob_in_hii)
print(hii_in_hob_test.vel, hii_in_hob_test.pos, hii_in_hob_test.time) # 0, 0, 1
cob_in_hob_test = change_in_tii_test1.translate_full('pos', hob_frame['hob'])
print(cob_in_hob_test.vel, cob_in_hob_test.pos, cob_in_hob_test.time) # 0, 0, 0 (actually hob in hob)
cob_in_hob_test = change_in_tii_test2.translate_full('pos', hob_frame['hob'], tii_frame['hob'])
print(cob_in_hob_test.vel, cob_in_hob_test.pos, cob_in_hob_test.time) # 0, 50, 0
cob_in_tii_test = change_in_tob_test.translate_full('pos', hob_frame['tii'], tob_frame['hob'])
print(cob_in_tii_test.vel, cob_in_tii_test.pos, cob_in_tii_test.time) # 50, -57.7, -1.15


home_in_tii_test1 = hob_frame['hob'].translate_full('time', hob_frame['tii'])
print(home_in_tii_test1.vel, home_in_tii_test1.pos, home_in_tii_test1.time)
home_in_tii_test2 = hob_frame['hii'].translate_full('time', tii_in_hii)
print(home_in_tii_test2.vel, home_in_tii_test2.pos, home_in_tii_test2.time)
change_in_tii_test2 = hob_frame['cob'].translate_full('time', hob_frame['tii'], hob_frame['hob'])
print(change_in_tii_test2.vel, change_in_tii_test2.pos, change_in_tii_test2.time)
change_in_tii_test3 = hob_frame['cii'].translate_full('time', tii_in_hii, hob_frame['hii'])
print(change_in_tii_test3.vel, change_in_tii_test3.pos, change_in_tii_test3.time)
change_in_tob_test = cob_in_tii.translate_full('time', hob_frame['tob'], tii_frame['hob'])
print(change_in_tob_test.vel, change_in_tob_test.pos, change_in_tob_test.time)
