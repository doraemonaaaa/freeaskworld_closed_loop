import quaternion
import time
import numpy as np

# ============ Coordinate System Transformation Functions ============
# Unity coordinate system: X-right, Y-up, Z-forward
# Mapper internal coordinate: X-right, Y-forward, Z-up (typical robotics convention)

def unity_translation(position):
    """
    Convert Unity position (X-right, Y-up, Z-forward) to mapper internal coords (X-right, Y-forward, Z-up).
    Unity: (x, y, z) -> Mapper: (x, z, y)
    """
    position = np.asarray(position, dtype=np.float64)
    # Unity Y-up, Z-forward -> Mapper Y-forward, Z-up
    return np.array([position[0], position[2], position[1]], dtype=np.float64)


def unity_rotation(rotation):
    """
    Convert Unity quaternion (x, y, z, w) to internal mapper rotation matrix.
    
    Unity coordinate system: X-right, Y-up, Z-forward (left-handed)
    Internal mapper:      X-right, Y-forward, Z-up (right-handed)
    
    Args:
        rotation: Unity quaternion as [x, y, z, w]
        
    Returns:
        3x3 rotation matrix in internal mapper coords
    """
    rotation = np.asarray(rotation, dtype=np.float64)
    qx, qy, qz, qw = rotation[0], rotation[1], rotation[2], rotation[3]

    # Unity uses left-handed system, we convert quaternion to match right-handed mapper
    # Swap Y and Z axes: Unity (X, Y, Z) -> Mapper (X, Z, Y)
    quat = quaternion.quaternion(qw, qx, qz, qy)  # w, x, z, y

    # Convert to 3x3 rotation matrix
    rotation_matrix = quaternion.as_rotation_matrix(quat)

    # Transformation matrix for coordinate conversion
    transform_matrix = np.array([
        [1, 0, 0],  # X -> X
        [0, 0, 1],  # Z -> Y
        [0, 1, 0]   # Y -> Z
    ])

    # Apply coordinate system transformation
    rotation_matrix = np.matmul(transform_matrix, rotation_matrix)

    return rotation_matrix

def habitat_translation(position):
    """
    Placeholder for Habitat coordinate translation.
    Habitat uses: X-right, Y-up, Z-backward (OpenGL convention)
    """
    position = np.asarray(position, dtype=np.float64)
    # Habitat: (x, y, z) -> Mapper: (x, -z, y)
    return np.array([position[0], -position[2], position[1]], dtype=np.float64)


def habitat_rotation(rotation):
    rotation_matrix = quaternion.as_rotation_matrix(rotation)
    transform_matrix = np.array([[1,0,0],
                                 [0,0,1],
                                 [0,1,0]])
    rotation_matrix = np.matmul(transform_matrix,rotation_matrix)
    return rotation_matrix

def identity_translation(position):
    """Pass-through translation (no coordinate conversion)."""
    return np.asarray(position, dtype=np.float64)


def identity_rotation(rotation):
    """
    Pass-through rotation - convert quaternion (x, y, z, w) to rotation matrix.
    """
    rotation = np.asarray(rotation, dtype=np.float64)
    qx, qy, qz, qw = rotation[0], rotation[1], rotation[2], rotation[3]
    quat = quaternion.quaternion(qw, qx, qy, qz)
    return quaternion.as_rotation_matrix(quat)