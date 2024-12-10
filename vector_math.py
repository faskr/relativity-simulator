import numpy as np

def vector(x, y, z):
    return np.array([x, y, z], dtype=np.float32)

# Scalar magnitude of vector
# How is this different from norm exactly? Is transpose the only difference?
def mag(v):
    return np.sqrt(np.dot(v, np.transpose(v)))

# Reshape a vector into two dimensions
def matrix_from_vec(v, transpose=False):
    if transpose:
        return v.reshape(v.size, 1)
    return v.reshape(1, v.size)

# Dot product of two vectors that creates a matrix
def matrix_from_vecs(u, v):
    return np.dot(matrix_from_vec(u, True), matrix_from_vec(v, False))

# Projection of u onto v: how long u is in direction of v and which way it's pointing relative to direction of v
def proj(u, v):
    if np.linalg.norm(v) == 0:
        return 0
    return np.dot(u, v) / np.linalg.norm(v)

# Absolute projection of u onto v: how long u is in direction of v and which way it's pointing in dimension of v
def abs_proj(u, v):
    return proj(u, np.abs(v))
