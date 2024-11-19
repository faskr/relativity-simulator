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

def plot_spacetime(ax_st, x, ct, axis_high, axis_low):
    ax_st.plot(axis_high, axis_low, 'b')
    ax_st.plot(axis_low, axis_high, 'b')
    ax_st.plot(x, x, 'k')
    ax_st.plot(x, -x, 'k')
    ax_st.plot(x, ct, 'r')
    ax_st.plot(np.transpose(x), np.transpose(ct), 'r')
    ax_st.set_xlabel('x')
    ax_st.set_ylabel('ct')
    ax_st.set_aspect('equal')

def create_spacetime_plot(p1, p2, t_steps, ax_st):
    # Minkowski spacetime values
    t_p1_in_obs = p1['time'] + t_steps
    t_p2_in_obs = p2['time'] + t_steps
    s_p1_in_obs = np.tile(p1['pos'], (5, 1)) + matrix_from_vecs(t_steps, p1['vel'])
    s_p2_in_obs = np.tile(p2['pos'], (5, 1)) + matrix_from_vecs(t_steps, p2['vel'])
    # Minkowski spacetime axes
    axis_scale = 1.2
    axis_high = c * t_steps * axis_scale
    axis_low = matrix_from_vecs(t_steps * axis_scale, p1['vel'])[:,0]
    # Plot
    x = np.array([s_p1_in_obs[:,0], s_p2_in_obs[:,0]])
    ct = np.array([c*t_p1_in_obs, c*t_p2_in_obs])
    plot_spacetime(ax_st, x, ct, axis_high, axis_low)

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
    create_spacetime_plot(p1, p2, t_steps, ax_md)
    # Plot settings
    plt.subplots_adjust(wspace=0.5, hspace=0.5)
    plt.grid()
    plt.show()
