from vector_math import *

# Speed of light
c = 100

# Velocity addition formula
def v_sum(v_a_in_b, v_b_in_c):
    return (v_a_in_b + v_b_in_c) / (1 + v_a_in_b * v_b_in_c / c**2)

def gamma(v):
    return 1 / np.sqrt(1 - v**2 / c**2)

def contract(x, v):
    return x / gamma(v)

def dilate(x, v):
    return x * gamma(v)

def t_phase(v_a_in_b, s_a):
    s_a_mag = proj(s_a, v_a_in_b)
    return mag(v_a_in_b) * s_a_mag / c**2

def s_phase(v_a_in_b, t_a):
    return v_a_in_b * t_a

# Transform foreign frame A to current frame B
def dilate_space(s_a, v_a_in_b, t_a):
    return dilate(s_a + s_phase(v_a_in_b, t_a), v_a_in_b)

# Transform foreign frame A to current frame B
def dilate_time(t_a, v_a_in_b, s_a):
    return dilate(t_a + t_phase(v_a_in_b, s_a), mag(v_a_in_b))

# Transform current frame A to foreign frame B
def contract_space(s_a, v_a_in_b, t_a):
    return contract(s_a + s_phase(v_a_in_b, t_a), v_a_in_b)

# Transform current frame A to foreign frame B
def contract_time(t_a, v_a_in_b, s_a):
    return contract(t_a + t_phase(v_a_in_b, s_a), mag(v_a_in_b))

def create_point(v, s, t):
    return {
        'vel': np.array(v, dtype=np.float32),
        'pos': np.array(s, dtype=np.float32),
        'time': t,
    }

# Calculate the velocity, position, and time of a point in a new frame, from known values in another frame
# This function assumes that the origins of both frames are the same event, which isn't always the case
def point_in_new_frame(v_old_in_new, point_in_old):
    return {
        'vel': v_old_in_new,
        'pos': dilate_space(point_in_old['pos'], v_old_in_new, point_in_old['time']),
        'time': dilate_time(point_in_old['time'], v_old_in_new, point_in_old['pos']),
    }

# Compute trajectory of a point/frame in another frame over time given its velocity and initial values in that frame
def trajectory(point, t_steps):
    return {
        'vel': point['vel'],
        'pos': np.tile(point['pos'], (t_steps.size, 1)) + matrix_from_vecs(t_steps, point['vel']),
        'time': point['time'] + t_steps,
    }
