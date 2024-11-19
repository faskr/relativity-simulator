import numpy as np
from vector_math import *
from relativity_math import *
from plot_functions import *

# ==== Setup ====

# Manual settings
v_trav_in_home_ob_x = c * 0.5
v_home_in_home_ib_x = c * -0.5
t_steps_obs = np.linspace(0, 0.99, 100)

# ==== Scene ====

# Set mover and observer in their own frames
trav_frame = {}
home_frame = {}
trav_frame['trav'] = create_point([0,0,0], [0,0,0], 0)
home_frame['home'] = create_point([0,0,0], [0,0,0], 0)
