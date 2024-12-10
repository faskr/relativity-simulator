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
class Point:
    def __init__(self, vel=[0,0,0], pos=[0,0,0], time=0):
        self.vel = np.array(vel, dtype=np.float32)
        self.pos = np.array(pos, dtype=np.float32)
        self.time = time
    
    # Calculate the velocity, position, and time of a point in a new frame, from known values in another frame
    # This function assumes that the origins of both frames are the same event, which isn't always the case
    def transform(self, v_old_in_new):
        return Point(
            v_transform(self.vel, v_old_in_new),
            s_transform(self.pos, v_old_in_new, self.time),
            t_transform(self.time, v_old_in_new, self.pos)
        )

    # Compute trajectory of a point/frame in another frame over time given its velocity and initial values in that frame
    def trajectory(self, t_steps):
        pos = np.tile(self.pos, (t_steps.size, 1)) + matrix_from_vecs(t_steps, self.vel)
        time = self.time + t_steps
        return Path.from_lists(pos, time)
    
    def trajectory_x(self, x_steps):
        pos = self.pos[0] + x_steps
        time = np.tile(self.time, (x_steps.size, 1)) + np.reshape(x_steps, (x_steps.size, 1)) / self.vel[0]
        return Path.from_lists(pos, time)

    def trajectory_in_interval(self, t_diff, tick_step=0.1, v_axis=0):
        t_steps = np.arange(0, t_diff, dilate(tick_step, self.vel[v_axis]))
        pos = np.tile(self.pos, (t_steps.size, 1)) + matrix_from_vecs(t_steps, self.vel)
        time = self.time + t_steps
        return Path.from_lists(pos, time)

    def convergence_time_with(self, other):
        s_diff_self_other = self.pos - other.pos
        v_diff_other_self = other.vel - self.vel
        return proj(s_diff_self_other, v_diff_other_self) / mag(v_diff_other_self)

class Path:
    def __init__(self, start_pos, start_time):
        self.pos = np.array([np.array(start_pos)])
        self.time = np.array([start_time])

    def from_lists(pos, time):
        path = Path(pos[0], time[0])
        path.pos = pos
        path.time = time
        return path

    def append(self, pos, time):
        self.pos = np.concatenate((self.pos, pos.reshape((1, 3))))
        self.time = np.concatenate((self.time, np.array([time])))

    def concatenate(self, more):
        pos = np.concatenate((self.pos, more.pos))
        time = np.concatenate((self.time, more.time))
        return Path.from_lists(pos, time)
    
    def concatenate_list(paths):
        pos = np.concatenate([path.pos for path in paths])
        time = np.concatenate([path.time for path in paths])
        return Path.from_lists(pos, time)

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

# TODO: have a variable that stores paths, and a function that appends to the paths given new velocities
class Frame():
    def __init__(self, name, pos=[0,0,0], time=0):
        self.name = name
        self.paths = {}
        self.points = {}
        self.point = Point([0,0,0], pos, time)
        self.path = Path(self.point.pos, self.point.time)
        self[name] = self.point
    
    def __setitem__(self, point_name, point):
        self.points[point_name] = point

    def __getitem__(self, point_name):
        return self.points[point_name]

    def add_point(self, name, vel=[0,0,0], pos=[0,0,0], time=0):
        self.points[name] = Point(vel, pos, time)

    def add_path(self, name, path):
        self.paths[name] = path

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