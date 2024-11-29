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

# What is dilation?
# Point: When a point is defined with a position and time in a foreign frame, that point has position and time in the current frame which are larger.
    # Transform without acceleration: If a point has certain coordinates in the frame of an object, what are its coordinates in the frame of another object?
    # Transform with acceleration: If a point has certain coordinates in the frame of an object, what are its coordinates in the frame of the same object moving at a different velocity?
# Length/lifespan: When a point is defined with a position or time in a simultaneous slice of time or space in the current frame, that point has a position or time that is larger in the foreign frame's slice.
# Note: duration or presence might be more apt terms than lifespan, since it's about how long an extent of space in one frame, is present in a point of space in the other.

# Transform foreign frame A to current frame B
#   Relative dilation: B observes a point as having a certain position in A, and a larger position in B
#   Absolute deceleration: A has decelerated to B; A is further from the frame of the object at s than B; the difference has objectively decreased; the increase in B = -v_a_in_b
#   Length dilation: B observes a difference of positions between two points that are measured in B and simultaneous in B, but observes a larger difference measured in A
def dilate_space(s_in_a, v_a_in_b, t_in_a):
    return dilate(s_in_a + s_phase(v_a_in_b, t_in_a), v_a_in_b)

# Transform foreign frame A to current frame B
def dilate_time(t_in_a, v_a_in_b, s_in_a):
    return dilate(t_in_a + t_phase(v_a_in_b, s_in_a), mag(v_a_in_b))

# What is contraction?
# Point: When a point is defined with a position and time in the current frame, that point has position and time in the foreign frame which are smaller.
    # See transform with vs. without acceleration above.
# Length/lifespan: When a point is defined with a position or time in a simultaneous slice of time or space in the foreign frame, that point has a position or time that is smaller in the current frame's slice.

# Transform current frame A to foreign frame B
#   Relative contraction: A observes a point as having a certain position in A, and a smaller position in B
#   Absolute acceleration: A has accelerated to B; B is further from the frame of the object at s than A; the difference has objectively increased; the increase in B = -v_a_in_b
#   Length contraction: A observes a difference of positions between two points that are measured in B and simultaneous in B, but observes a smaller difference measured in A
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
