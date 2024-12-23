from vector_math import *
from relativity import *

class Event:
    def __init__(self, pos=[0,0,0], time=0):
        self.pos = vector(pos)
        self.time = time
    
    def transform(self, v_old_in_new):
        return Event(
            s_transform(self.pos, v_old_in_new, self.time),
            t_transform(self.time, v_old_in_new, self.pos)
        )

# TODO: Instead of Point, have Event(pos, time), and maybe something like Capture(vel, pos, time) or Waypoint, since it's part of a path
# A point isn't supposed to have velocity; objects have velocity
# TODO: Probably create a Line class, and have subclasses called Trajectory and Simultaneity; maybe give the Capture class variables of Event, Trajectory, and Simultaneity types

# TODO: Start by simply changing the name of this class to Capture, and probably make it a subclass of Event
class Point:
    def __init__(self, vel=[0,0,0], pos=[0,0,0], time=0):
        self.vel = np.array(vel, dtype=np.float32)
        self.pos = np.array(pos, dtype=np.float32)
        self.time = time
    
    # Calculate point when origin of old is offset to coincide with origin of new frame
    def offset(self, new_in_old):
        return Point(
            self.vel,
            self.pos - new_in_old.pos, # old pos relative to new pos
            self.time - new_in_old.time # old time relative to new time
        )
    
    # Calculate the velocity, position, and time of a point in a new frame, from known values in another frame
    def transform(self, new_in_old):
        p_in_offset_old = self.offset(new_in_old)
        v_old_in_new = -new_in_old.vel
        return Point(
            v_transform(p_in_offset_old.vel, v_old_in_new),
            s_transform(p_in_offset_old.pos, v_old_in_new, p_in_offset_old.time),
            t_transform(p_in_offset_old.time, v_old_in_new, p_in_offset_old.pos)
        )
    
    def transform_no_offset(self, new_in_old):
        v_old_in_new = -new_in_old.vel
        return Point(
            v_transform(new_in_old.vel, v_old_in_new),
            s_transform(new_in_old.pos, v_old_in_new, new_in_old.time),
            t_transform(new_in_old.time, v_old_in_new, new_in_old.pos)
        )
    
    # Intuitive way to see down translation: rotate axis of specified dimension, around intersection at foreign frame axis, into foreign frame, and shift that axis so that the point at the specified amount away hits the non-specified foreign axis
    # Intuitive purpose: find where a trajectory intersects a certain time in a foreign frame, or when the simultaneity intersects a certain position
    def translate_one_way(self, dim_type, direction, new_in_old, at_coord):
        # Translate up: dilation formula with phase, from motion to rest (*will* end up at rest)
        # Translate down: contraction formula with phase, from rest to motion (*must* be from rest frame)
        if direction == 'down':
            assert not self.vel.any()
        p_offset_in_old = self.offset(new_in_old)
        v_old_in_new = -new_in_old.vel
        v_new = vector(0, 0, 0) if direction == 'up' else v_old_in_new if direction == 'down' else None
        if dim_type == 'pos': # Intersection of new line of simultaneity with the same trajectory
            if direction == 'up':
                s_new = s_transform(p_offset_in_old.pos, v_old_in_new, p_offset_in_old.time)
            elif direction == 'down':
                s_new = transform_trajectory_t_s(p_offset_in_old.pos, v_old_in_new, at_coord)
            t_new = at_coord
        elif dim_type == 'time': # Intersection of new trajectory with the same line of simultaneity
            s_new = at_coord
            if direction == 'up':
                t_new = t_transform(p_offset_in_old.time, v_old_in_new, p_offset_in_old.pos)
            elif direction == 'down':
                t_new = transform_simultaneity_s_t(p_offset_in_old.time, v_old_in_new, at_coord)
        return Point(v_new, s_new, t_new)

    # Calculates *one* of the following:
    # s_new = (s_rest - v_down*t_rest) / gamma_down = ([s_old*gamma_up - v_up*t_old*gamma_up] - v_down*t_old) / gamma_down
    # t_new = (t_rest - v_down*s_rest/c^2) / gamma_down = ([t_old*gamma_up - v_up*s_old*gamma_up/c^2] - v_down*s_old/c^2) / gamma_down
    def translate_full(self, dim_type, new_in_rest, rest_in_old=None):
        at_coord = self.time if dim_type == 'pos' else self.pos if dim_type == 'time' else None
        # Translate up to rest frame
        rest_in_old = self if rest_in_old == None else rest_in_old # By default, p is at the origin of rest, but this is not necessarily the case
        p_in_rest = self.translate_one_way(dim_type, 'up', rest_in_old, at_coord)
        # Translate down to new frame
        return p_in_rest.translate_one_way(dim_type, 'down', new_in_rest, at_coord)

    # Compute trajectory of a point/frame in another frame over time given its velocity and initial values in that frame
    def trajectory(self, t_steps):
        pos = np.tile(self.pos, (t_steps.size, 1)) + matrix_from_vecs(t_steps, self.vel)
        time = self.time + t_steps
        return Path.from_lists(pos, time)
    
    def trajectory_x(self, x_steps):
        pos = self.pos[0] + x_steps
        time = np.tile(self.time, (x_steps.size, 1)) + np.reshape(x_steps, (x_steps.size, 1)) / self.vel[0]
        return Path.from_lists(pos, time)

    def trajectory_in_interval(self, other, tick_step=0.1, v_axis=0):
        duration = other.time - self.time
        t_steps = np.arange(0, duration, dilate(tick_step, self.vel[v_axis]))
        return self.trajectory(t_steps)

    def convergence_time_with(self, other):
        v_diff_self_other = self.vel - other.vel
        s_diff_other_self = other.pos - self.pos
        t_diff_other_self = other.time - self.time
        s_diff_phase = t_diff_other_self * proj(other.vel, -v_diff_self_other)
        return (proj(s_diff_other_self, v_diff_self_other) + s_diff_phase) / mag(v_diff_self_other)

class Path:
    def __init__(self, start_pos, start_time):
        self.pos = np.array([np.array(start_pos)])
        self.time = np.array([start_time])

    def from_lists(pos, time):
        path = Path(pos[0], time[0])
        path.pos = pos
        path.time = time
        return path

    def add_point(self, pos, time):
        #print(pos, time)
        more = Path(pos, time)
        self.pos = np.concatenate((self.pos, more.pos))
        self.time = np.concatenate((self.time, more.time))
        return Path.from_lists(more.pos, more.time)

    def concatenate(self, more):
        pos = np.concatenate((self.pos, more.pos))
        time = np.concatenate((self.time, more.time))
        return Path.from_lists(pos, time)
    
    def concatenate_list(paths):
        pos = np.concatenate([path.pos for path in paths])
        time = np.concatenate([path.time for path in paths])
        return Path.from_lists(pos, time)

    def add_segment(self, p1, p2):
        segment = p1.trajectory_in_interval(p2)
        new_path = self.concatenate(segment)
        self.pos = new_path.pos
        self.time = new_path.time
        return segment

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
        trajectories.append(Path.from_lists([s_segment], [t_segment])) # Endpoint that is intended to be excluded from tick marks
        return Path.concatenate_list(trajectories)

    def transform_path(self, new_in_old):
        new_pos = np.zeros(self.pos.shape)
        new_time = np.zeros(self.time.shape)
        v_old_in_new = -new_in_old.vel
        for i in range(len(self.pos)): # TODO: abstract away offset at least
            pos_offset = self.pos[i] - new_in_old.pos
            time_offset = self.time[i] - new_in_old.time
            new_pos[i, :] = s_transform(pos_offset, v_old_in_new, time_offset)
            new_time[i] = t_transform(time_offset, v_old_in_new, pos_offset)
        return Path.from_lists(new_pos, new_time)

# TODO: have a variable that stores paths, and a function that appends to the paths given new velocities
class Frame():
    def __init__(self, name):
        self.name = name
        self.point = Point([0,0,0], [0,0,0], 0)
        self.points = {}
        self.points[name] = self.point
        self.path = Path(self.point.pos, self.point.time)
        self.paths = {}
    
    def __setitem__(self, point_name, point):
        self.points[point_name] = point

    def __getitem__(self, point_name):
        return self.points[point_name]

    def add_point(self, name, vel=[0,0,0], pos=[0,0,0], time=0):
        self.points[name] = Point(vel, pos, time)

    def add_path(self, name, point_name):
        self.paths[name] = Path(self[point_name].pos, self[point_name].time)

    def in_frame(self, other):
        if self.name in other:
            return other[self.name]
        if other.name in self:
            v_self_in_other = -self[other.name].vel
            other[self.name] = self[other.name].new_frame(v_self_in_other)
            return other[self.name]
        raise Exception(f'Velocity between frames {self.name} and {other.name} has not been defined')
    
    def inertial_step(self, old_name, new_name, t_diff, v_new):
        s_new = self[old_name].pos + self[old_name].vel * t_diff
        t_new = self[old_name].time + t_diff
        self[new_name] = Point(vel=v_new, pos=s_new, time=t_new)

    def inertial_step_back(self, old_name, new_name, t_diff, v_new):
        s_new = self[old_name].pos + v_new * t_diff
        t_new = self[old_name].time + t_diff
        self[new_name] = Point(vel=v_new, pos=s_new, time=t_new)
