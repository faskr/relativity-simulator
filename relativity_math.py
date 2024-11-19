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

def t_phase(v, s):
    s_mag = proj(s, v)
    return mag(v) * s_mag / c**2

def s_phase(v, t):
    return v * t

# Transform foreign frame to current frame
def dilate_space(s, v, t):
    return dilate(s + s_phase(v, t), v)

# Transform foreign frame to current frame
def dilate_time(t, v, s):
    return dilate(t + t_phase(v, s), mag(v))

# Transform current frame to foreign frame
def contract_space(s, v, t):
    return contract(s + s_phase(v, t), v)

# Transform current frame to foreign frame
def contract_time(t, v, s):
    return contract(t + t_phase(v, s), mag(v))

# Calculate the velocity, position, and time of a point in a new frame, from known values in another frame
def point_in_new_frame(v_old_in_new, old_frame, point_name):
    return {
        'vel': v_old_in_new,
        'pos': dilate_space(old_frame[point_name]['pos'], v_old_in_new, old_frame[point_name]['time']),
        'time': dilate_time(old_frame[point_name]['time'], v_old_in_new, old_frame[point_name]['pos']),
    }

def create_point(v, s, t):
    return {
        'vel': np.array(v, dtype=np.float32),
        'pos': np.array(s, dtype=np.float32),
        'time': t,
    }
