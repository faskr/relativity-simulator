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

# Calculate offset in time due to location in space in direction of velocity
# Over space in the direction of movement, is time going forward or backward?
def t_phase(v_a_in_b, s_in_a):
    s_in_a_mag = proj(s_in_a, v_a_in_b)
    v_a_in_b_mag = mag(v_a_in_b) # Velocity is always positive in its own direction
    return v_a_in_b_mag * s_in_a_mag / c**2

# Calculate offset in all spatial dimensions due to passage of time
# Over time, is position increasing or decreasing?
def s_phase(v_a_in_b, t_in_a):
    return v_a_in_b * t_in_a

# Transform foreign frame A to current frame B
#   Relative dilation: B observes a point as having a certain position in A, and a larger position in B
#   Absolute deceleration: A has decelerated to B, and the absolute difference between A and the frame of the object at s has decreased by v_a_in_b
def dilate_space(s_in_a, v_a_in_b, t_in_a):
    return dilate(s_in_a + s_phase(v_a_in_b, t_in_a), v_a_in_b)

# Transform foreign frame A to current frame B
def dilate_time(t_in_a, v_a_in_b, s_in_a):
    return dilate(t_in_a + t_phase(v_a_in_b, s_in_a), mag(v_a_in_b))

# Transform current frame A to foreign frame B
#   Relative contraction: A observes a point as having a certain position in A, and a smaller position in B
#   Absolute acceleration: A has accelerated to B, and the absolute difference between A and the frame of the object at s has increased by v_a_in_b
def contract_space(s_in_a, v_a_in_b, t_in_a):
    return contract(s_in_a + s_phase(v_a_in_b, t_in_a), v_a_in_b)

# Transform current frame A to foreign frame B
def contract_time(t_in_a, v_a_in_b, s_in_a):
    return contract(t_in_a + t_phase(v_a_in_b, s_in_a), mag(v_a_in_b))

class Point:
    def __init__(self, vel, pos, time):
        self.vel = np.array(vel, dtype=np.float32)
        self.pos = np.array(pos, dtype=np.float32)
        self.time = time
    
    # Calculate the velocity, position, and time of a point in a new frame, from known values in another frame
    # This function assumes that the origins of both frames are the same event, which isn't always the case
    def new_frame(self, v_old_in_new):
        return Point(
            v_old_in_new,
            dilate_space(self.pos, v_old_in_new, self.time),
            dilate_time(self.time, v_old_in_new, self.pos)
        )

    # Compute trajectory of a point/frame in another frame over time given its velocity and initial values in that frame
    def trajectory(self, t_steps):
        pos = np.tile(self.pos, (t_steps.size, 1)) + matrix_from_vecs(t_steps, self.vel)
        time = self.time + t_steps
        return Path(pos, time)
    
    def trajectory_x(self, x_steps):
        pos = self.pos[0] + x_steps
        time = np.tile(self.time, (x_steps.size, 1)) + np.reshape(x_steps, (x_steps.size, 1)) / self.vel[0]
        return Path(pos, time)

class Path:
    def __init__(self, pos, time):
        self.pos = pos
        self.time = time

    def concatenate(self, more):
        pos = np.concatenate((self.pos, more.pos))
        time = np.concatenate((self.time, more.time))
        return Path(pos, time)
