""" Utilities to operate on structured array files directly,
such as the ``pc_data`` in ``PointCloud`` objects.
"""


import numpy as np


def transform_xyz(T_a_b, xyz):
    """ Transforms an Nx3 array xyz in frame a to frame b
    T_a_b is a 4x4 matrix s.t. xyz_b = T_a_b * xyz_a
    conversely, T_a_b is the pose a in the frame b
    """
    # xyz in frame a, homogeneous
    xyz1_a = np.vstack([xyz.T, np.ones((1, xyz.shape[0]))])
    # xyz in b frame
    xyz1_b = np.dot(T_a_b, xyz1_a)
    xyz_b = np.ascontiguousarray((xyz1_b[:3]).T)
    return xyz_b


def transform_cloud_array(T_a_b, pc_data):
    """ transforms structured array. looks for xyz and xyz_origin """
    xyz = get_xyz_array(pc_data, dtype=pc_data.dtype[0])
    xyz_b = transform_xyz(T_a_b, xyz)
    pc_data['x'] = xyz_b[:, 0]
    pc_data['y'] = xyz_b[:, 1]
    pc_data['z'] = xyz_b[:, 2]
    if 'x_origin' in pc_data:
        xyz_origin = get_xyz_viewpoint_array(pc_data, dtype=pc_data.dtype[0])
        xyz_origin_b = transform_xyz(T_a_b, xyz_origin)
        pc_data['x_origin'] = xyz_origin_b[:, 0]
        pc_data['y_origin'] = xyz_origin_b[:, 1]
        pc_data['z_origin'] = xyz_origin_b[:, 2]
    return pc_data


def flip_around_x(pc_data):
    """ flip a structured array around x, in place"""
    pc_data['y'] = -pc_data['y']
    pc_data['z'] = -pc_data['z']

    if 'x_origin' in pc_data:
        pc_data['y_origin'] = -pc_data['y_origin']
        pc_data['z_origin'] = -pc_data['z_origin']


def get_xyz_array(pc_data, dtype=np.float32):
    """ get Nx3 array from structured array """
    if pc_data.ndim == 2 and pc_data.shape[0] == 1:
        pc_data = pc_data.squeeze()
    xyz = np.empty((len(pc_data), 3), dtype=dtype)
    xyz[:, 0] = pc_data['x']
    xyz[:, 1] = pc_data['y']
    xyz[:, 2] = pc_data['z']
    return xyz


def get_xyz_viewpoint_array(pc_data, dtype=np.float32):
    """ get Nx3 array from structured array """
    if pc_data.ndim == 2 and pc_data.shape[0] == 1:
        pc_data = pc_data.squeeze()
    xyz = np.empty((len(pc_data), 3), dtype=dtype)
    xyz[:, 0] = pc_data['x_origin']
    xyz[:, 1] = pc_data['y_origin']
    xyz[:, 2] = pc_data['z_origin']
    return xyz


def get_xyzl_array(pc_data, dtype=np.float32):
    if pc_data.ndim == 2 and pc_data.shape[0] == 1:
        pc_data = pc_data.squeeze()
    xyzl = np.empty((len(pc_data), 4), dtype=dtype)
    xyzl[:, 0] = pc_data['x']
    xyzl[:, 1] = pc_data['y']
    xyzl[:, 2] = pc_data['z']
    xyzl[:, 3] = pc_data['label']
    return xyzl
