from vector_math import *

def convergence_time(p1, p2):
    s_diff_p1_p2 = p1.pos - p2.pos
    v_diff_p2_p1 = p2.vel - p1.vel
    return proj(s_diff_p1_p2, v_diff_p2_p1) / mag(v_diff_p2_p1)
