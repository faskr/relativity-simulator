from vector_math import *

# Speed of light
c = 100

# Velocity addition formula; rename to transform_velocity?
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
def transform_space(s_p_in_a, v_a_in_b, t_p_in_a):
# TODO: possibly use "- phase(v_b_in_a)" instead
    return dilate(s_p_in_a + s_phase(v_a_in_b, t_p_in_a), v_a_in_b)

# Transform foreign frame A to current frame B
def transform_time(t_p_in_a, v_a_in_b, s_p_in_a):
    return dilate(t_p_in_a + t_phase(v_a_in_b, s_p_in_a), mag(v_a_in_b))

# What is contraction and when does it apply?
# Point: When a point is defined with a position or time in the current frame which is measured in a foreign frame's axis of space or time, that position or time is smaller in the foreign frame.
    # See transform with vs. without acceleration above.
# Length/lifespan: When a point is defined with a position or time in the foreign frame which is measured in the current frame's axis of space or time, that position or time is smaller in the current frame.

# Transform current frame A to foreign frame B
#   Relative contraction: A observes a point as having a certain position in A, and a smaller position in B
#   Absolute acceleration: A has accelerated to B; B is further from the frame of the object at s than A; the difference has objectively increased; the increase in B = -v_a_in_b
#   Length contraction: A observes a difference of positions between two points that are measured in B and simultaneous in B, but observes a smaller difference measured in A
def contract_space(s_in_a, v_a_in_b, t_in_b):
    return contract(s_in_a, v_a_in_b) + s_phase(v_a_in_b, t_in_b)

# Transform current frame A to foreign frame B
def contract_time(t_in_a, v_a_in_b, s_in_b):
    return contract(t_in_a, mag(v_a_in_b)) + t_phase(v_a_in_b, s_in_b)

class Frame():
    points = {}
    def __init__(self, name, pos=[0,0,0], time=0):
        self.name = name
        self.point = Point([0,0,0], pos, time)
        self[name] = self.point
    
    def __setitem__(self, point_name, point):
        self.points[point_name] = point

    def __getitem__(self, point_name):
        return self.points[point_name]

    def in_frame(self, other):
        if self.name in other:
            return other[self.name]
        if other.name in self:
            other[self.name] = transform(self, -self[other.name].vel)
            return other[self.name]
        raise Exception(f'Velocity between frames {self.name} and {other.name} has not been defined')

# TODO: Instead of Point, have Event(pos, time), and maybe something like Capture(vel, pos, time)
class Point:
    def __init__(self, vel=[0,0,0], pos=[0,0,0], time=0):
        self.vel = np.array(vel, dtype=np.float32)
        self.pos = np.array(pos, dtype=np.float32)
        self.time = time
    
    # Calculate the velocity, position, and time of a point in a new frame, from known values in another frame
    # This function assumes that the origins of both frames are the same event, which isn't always the case
    def new_frame(self, v_old_in_new):
        return Point(
            v_sum(self.vel, v_old_in_new),
            transform_space(self.pos, v_old_in_new, self.time),
            transform_time(self.time, v_old_in_new, self.pos)
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
    
    def concatenate_list(paths):
        pos = np.concatenate([path.pos for path in paths])
        time = np.concatenate([path.time for path in paths])
        return Path(pos, time)
    
    def create_path(s_start, t_start, segment_durations, segment_velocities, tick_step=0.1, v_axis=0):
        t_segment = t_start
        s_segment = s_start.copy()
        trajectories = []
        for segment in range(len(segment_durations)):
            t_steps = np.arange(0, segment_durations[segment], dilate(tick_step, segment_velocities[segment][v_axis]))
            frame = Point(segment_velocities[segment], s_segment, t_segment)
            trajectories.append(frame.trajectory(t_steps))
            t_segment += segment_durations[segment]
            s_segment += segment_velocities[segment] * segment_durations[segment]
        trajectories.append(Path([s_segment], [t_segment])) # Endpoint that is intended to be excluded from tick marks
        return Path.concatenate_list(trajectories)
