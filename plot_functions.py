import numpy as np
import matplotlib.pyplot as plt
from relativity_math import *

def plot_3d_box(ax_3d, position, length):
    # Define coordinates and grids for box
    y_range = np.linspace(position[1], position[1] + length[1], 2)
    z_range = np.linspace(position[2], position[2] + length[2], 2)
    x_range = np.linspace(position[0], length[0], 2)
    sq_y, sq_z = np.meshgrid(y_range, z_range)
    rc_xy, rc_y = np.meshgrid(x_range, y_range)
    rc_xz, rc_z = np.meshgrid(x_range, z_range)
    ones = np.ones((2, 2))
    # Plot box
    ax_3d.plot_surface(rc_xy, rc_y, ones*position[2], color='r') # z = 0
    ax_3d.plot_surface(rc_xy, rc_y, ones*position[2] + length[2], color='r') # z = 1
    ax_3d.plot_surface(rc_xz, ones*position[1], rc_z, color='r') # y = 0
    ax_3d.plot_surface(rc_xz, ones*position[1] + length[1], rc_z, color='r') # y = 1
    ax_3d.plot_surface(ones*position[0], sq_y, sq_z, color='r') # x = 0
    ax_3d.plot_surface(ones*length[0], sq_y, sq_z, color='r') # x = L0
    ax_3d.set_xlabel('x')
    ax_3d.set_ylabel('y')
    ax_3d.set_zlabel('z')
    ax_3d.set_aspect('equal')

def plot_steps(ax_st, trajectories, axis):
    x_steps = np.stack([t['pos'][:, axis] for t in trajectories], axis=0)
    ct_steps = np.stack([c * t['time'] for t in trajectories], axis=0)
    ax_st.plot(x_steps, ct_steps, 'r')
    ax_st.set_xlabel('x')
    ax_st.set_ylabel('ct')
    ax_st.set_aspect('equal')
    plt.grid()

def plot_trajectories(ax_st, trajectories, axis):
    x_trajs = np.stack([t['pos'][:, axis] for t in trajectories], axis=1)
    ct_trajs = np.stack([c * t['time'] for t in trajectories], axis=1)
    ax_st.plot(x_trajs, ct_trajs, 'r')
    ax_st.set_xlabel('x')
    ax_st.set_ylabel('ct')
    ax_st.set_aspect('equal')
    plt.grid()

def plot_spacetime(ax_st, trajectories, v_axis=0, axes=None, do_cones=True, do_step=False, do_traj=True):
    x_steps = np.stack([t['pos'][:, v_axis] for t in trajectories], axis=0)
    ct_steps = np.stack([c * t['time'] for t in trajectories], axis=0)
    if axes:
        ax_st.plot(axes[0], axes[1], 'b')
        ax_st.plot(axes[1], axes[0], 'b')
    if do_cones:
        coord_list = np.array([np.min(x_steps), np.max(x_steps)])
        ax_st.plot(coord_list, coord_list, 'k')
        ax_st.plot(coord_list, -coord_list, 'k')
    if do_step:
        ax_st.plot(x_steps, ct_steps, 'r')
    if do_traj:
        ax_st.plot(np.transpose(x_steps), np.transpose(ct_steps), 'r')
    ax_st.set_xlabel('x')
    ax_st.set_ylabel('ct')
    ax_st.set_aspect('equal')
    plt.grid()

def plot_spacetime_2_points(p1, p2, t_steps, ax_st):
    # Minkowski spacetime values
    traj_p1 = trajectory(p1, t_steps)
    traj_p2 = trajectory(p2, t_steps)
    # Minkowski spacetime axes
    axis_scale = 1.2
    axis_high = c * t_steps * axis_scale
    axis_low = matrix_from_vecs(t_steps * axis_scale, p1['vel'])[:,0]
    # Plot
    plot_spacetime(ax_st, (traj_p1, traj_p2), axes=(axis_high, axis_low), do_step=True)

def create_relativity_subplots(p1, p2, length, t_steps):
    # Figure
    fig = plt.figure()
    fig.suptitle('Mover in Observer Frame')
    # Plot mover
    ax_3d = fig.add_subplot(1,2,1,projection='3d')
    ax_3d.set_title('Appearance')
    plot_3d_box(ax_3d, p1['pos'], length)
    # Minkowski spacetime
    ax_md = fig.add_subplot(1,2,2)
    ax_md.set_title('Minkowski Spacetime')
    plot_spacetime_2_points(p1, p2, t_steps, ax_md)
    # Plot settings
    plt.subplots_adjust(wspace=0.5, hspace=0.5)
    plt.show()
