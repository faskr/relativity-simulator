from vector_math import *

# Speed of light
c = 100

# Velocity addition formula; TODO: rename to transform_velocity?
def v_transform(v_a_in_b, v_b_in_c):
    return (v_a_in_b + v_b_in_c) / (1 + v_a_in_b * v_b_in_c / c**2)

def gamma(v):
    return 1 / np.sqrt(1 - v**2 / c**2)

def contract(x, v):
    return x / gamma(v)

def dilate(x, v):
    return x * gamma(v)

# Calculate time between simultaneities due to location in space in direction of velocity
# Over space in the direction of movement, is time going forward or backward?
def t_phase_simultaneities(v_a_in_b, s_in_a):
    s_in_a_mag = proj(s_in_a, v_a_in_b)
    v_a_in_b_mag = mag(v_a_in_b) # Velocity is always positive in its own direction
    return v_a_in_b_mag * s_in_a_mag / c**2

# Calculate time until intersection of trajectories given distance between them
# Warning: if mag(v) is zero, output will be nan
def t_phase_trajectories(v_a_in_b, s_in_a):
    s_in_a_mag = proj(s_in_a, v_a_in_b)
    v_a_in_b_mag = mag(v_a_in_b) # Velocity is always positive in its own direction
    return s_in_a_mag / v_a_in_b_mag

# Calculate offset in all spatial dimensions due to passage of time
# Over time, is position increasing or decreasing?
def s_phase_trajectories(v_a_in_b, t_in_a):
    return v_a_in_b * t_in_a

# Calculate space between point where simultaneities are the given duration apart, and intersection of simultaneities, in direction of velocity
# Warning: for any dimensions of v = 0, the simultaneities are parallel and so the phase cannot be calculated from time
def s_phase_simultaneities(v_a_in_b, t_in_a):
    return t_in_a * c**2 / v_a_in_b

# TODO: Are these comments true? Are they necessary or out of date?

# What is dilation and when does it apply?
# Point: When a point is defined with a position or time in a foreign frame and measured in that frame's axis of space or time (not considering phase), that point's position or time is larger in the current frame.
    # Transform without acceleration: If a point has certain coordinates in the frame of an object, what are its coordinates in the frame of another object?
    # Transform with acceleration: If a point has certain coordinates in the frame of an object, what are its coordinates in the frame of the same object moving at a different velocity?
# Length/lifespan: When a point is defined with a position or time in the current frame and measured in that frame's axis of space or time (not considering phase), that point's position or time is larger in the foreign frame.
# Note: duration or presence might be more apt terms than lifespan, since it's about how long an extent of space in one frame, is present in a point of space in the other.

# Transform foreign frame A to current frame B
#   Relative dilation: B observes a point as having a certain position in A, and a larger position in B
#   Absolute deceleration: A has decelerated to B; A is further from the frame of the object at s than B; the difference has objectively decreased; the increase in B = -v_a_in_b
#   Length dilation: B observes a difference of positions between two points that are measured in B and simultaneous in B, but observes a larger difference measured in A
def s_transform(s_p_in_a, v_a_in_b, t_p_in_a):
# TODO: possibly use "- phase(v_b_in_a)" instead
    return dilate(s_p_in_a + s_phase_trajectories(v_a_in_b, t_p_in_a), v_a_in_b)

# Transform foreign frame A to current frame B
def t_transform(t_p_in_a, v_a_in_b, s_p_in_a):
    return dilate(t_p_in_a + t_phase_simultaneities(v_a_in_b, s_p_in_a), mag(v_a_in_b))

# What is contraction and when does it apply?
# Point: When a point is defined with a position or time in the current frame which is measured in a foreign frame's axis of space or time, that position or time is smaller in the foreign frame.
    # See transform with vs. without acceleration above.
# Length/lifespan: When a point is defined with a position or time in the foreign frame which is measured in the current frame's axis of space or time, that position or time is smaller in the current frame.

# Transform current frame A to foreign frame B
#   Relative contraction: A observes a point as having a certain position in A, and a smaller position in B
#   Absolute acceleration: A has accelerated to B; B is further from the frame of the object at s than A; the difference has objectively increased; the increase in B = -v_a_in_b
#   Length contraction: A observes a difference of positions between two points that are measured in B and simultaneous in B, but observes a smaller difference measured in A

# Calculates a rest trajectory in the new frame that the trajectory in the old frame of an object moving in the new frame (& at rest in old) intersects at the given time, given a source position in the old rest frame
# Useful for some operations such as space translation, i.e. what other points lie on the trajectory of the object at this point?
# E.g. In translation, used to find where a trajectory intersects a certain time in a foreign frame
def transform_trajectory_t_s(s_traj_in_a, v_a_in_b, t_p_in_b):
    return contract(s_traj_in_a, v_a_in_b) + s_phase_trajectories(v_a_in_b, t_p_in_b)

# E.g. In translation, used to find when a trajectory intersects a certain position in a foreign frame
def transform_trajectory_s_t(s_traj_in_a, v_b_in_a, s_p_in_b):
    return t_phase_trajectories(v_b_in_a, contract(s_traj_in_a, v_b_in_a) - s_p_in_b)

# Transform current frame A to foreign frame B

# Calculates a line of simultaneity in the new frame that the simultaneity line in the old frame of an object moving in the new frame (& at rest in old) intersects at the given position, given its time in the old rest frame
# Useful for some operations such as time translation, i.e. what other points are simultaneous with the object at this point?
# E.g. In translation, used to find when a line of simultaneity intersects a certain position in a foreign frame
def transform_simultaneity_s_t(t_simul_in_a, v_a_in_b, s_p_in_b):
    return contract(t_simul_in_a, mag(v_a_in_b)) + t_phase_simultaneities(v_a_in_b, s_p_in_b)

# E.g. In translation, used to find where a line of simultaneity intersects a certain time in a foreign frame
def transform_simultaneity_t_s(t_simul_in_a, v_b_in_a, t_p_in_b):
    return s_phase_simultaneities(v_b_in_a, contract(t_simul_in_a, mag(v_b_in_a)) - t_p_in_b)
