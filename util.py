# from tkinter import Y
import numpy as np
import math
import math3d as m3d
import os

#limit                
gripper_limits = {'joint_x': [-0.1, 0.1], 
                'joint_y': [-0.1, 0.1], 
                'joint_z': [0, 0.1],
                'joint_roll': [-45, 45],
                'joint_pitch': [-45, 45],
                'joint_yaw': [-90, 90],
                'joint_open':[0,1],
                'joint_distance':[0,0.3],
                'steps':[0,4]}

objects_limits = {'o_x': [-0.07, 0.07], 
                'o_y': [-0.07, 0.07], 
                'o_z': [0, 0.1],
                'o_roll': [-1.57, 1.57],
                'o_pitch': [-1.57, 1.57],
                'o_yaw': [-1.57, 1.57]}
def next_path(path_pattern):
    """
    Finds the next free path in an sequentially named list of files
    e.g. path_pattern = 'file-%s.txt':
    file-1.txt
    file-2.txt
    file-3.txt
    Runs in log(n) time where n is the number of existing files in sequence
    """
    i = 1

    # First do an exponential search
    while os.path.exists(path_pattern % i):
        i = i * 2

    # Result lies somewhere in the interval (i/2..i]
    # We call this interval (a..b] and narrow it down until a + 1 = b
    a, b = (i // 2, i)
    while a + 1 < b:
        c = (a + b) // 2  # interval midpoint
        a, b = (c, b) if os.path.exists(path_pattern % c) else (a, c)

    return path_pattern % b
def discretize(value, possibilities):
    closest_value = possibilities[0]
    for i in range(len(possibilities)):
        if abs(value - possibilities[i]) < abs(value - closest_value):
            closest_value = possibilities[i]
    return closest_value


def rescale(x, x_min, x_max, y_min, y_max):
    return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min


def normalize(x, old_range):
    return rescale(x, old_range[0], old_range[1], -1, 1)

def normalize_01(x, old_range):
    return rescale(x, old_range[0], old_range[1], 0, 1)

def unnormalize_01(x, new_range):
    return rescale(x, 0, 1, new_range[0], new_range[1])

def unnormalize(x, new_range):
    return rescale(x, -1, 1, new_range[0], new_range[1])


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def T_rotatez(rx, ry, rz, yaw):
    grip_rot = m3d.Transform()
    grip_rot.pos = (0, 0, 0)
    grip_rot.orient.rotate_zb(yaw)  # yaw
    grip_matrix = grip_rot.get_matrix()
    gripper_base_start_pos = np.array([rx, ry, rz, 1]).reshape(4, 1)
    g_tool = np.matmul(grip_matrix, gripper_base_start_pos)
    return g_tool[0:3].tolist()


def gripper_orn_to_world(pitch, roll, yaw):
    grip_rot = m3d.Transform()
    grip_rot.pos = (0, 0, 0)
    grip_rot.orient.rotate_yb(roll)  # roll
    grip_rot.orient.rotate_xb(pitch)  # pitch
    grip_rot.orient.rotate_zb(yaw)  # yaw
    grip_matrix = grip_rot.get_matrix()
    robot_Orn = R.from_matrix(grip_matrix[:3, :3]).as_quat()
    return robot_Orn


def rotate_point(point, theta):
    return np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ]) @ point


def is_point_below_line(point, line_point, line_angle, n_offset=0):
    k = math.tan(line_angle)
    n = line_point[1] - k * line_point[0] + n_offset
    return point[1] < k * point[0] + n


def is_point_inside_rectangle(point, center, width, length, orientation):
    point = np.array(point)
    center = np.array(center)
    orientation = np.array(orientation)
    point_rot = rotate_point(point - center, -orientation)
    return point_rot[0] > -length/2 and point_rot[0] < length/2 and point_rot[1] > -width/2 and point_rot[1] < width/2


def normalize_angle(theta):
    if theta > math.pi:
        theta -= 2 * math.pi
    elif theta < -math.pi:
        theta += 2 * math.pi
    return theta


def are_angles_close(theta1, theta2, threshold):
    return abs(normalize_angle(theta1) - normalize_angle(theta2)) < threshold


if __name__ == "__main__":
    # Perform some tests
    assert is_point_below_line([0, 0], [0, 1], 0)
    assert not is_point_below_line([0, 2], [0, 1], 0)
    assert is_point_below_line([5, 2], [0, 1], math.pi / 4)
    assert not is_point_below_line([1, 2], [0, 1], math.pi / 4)
    assert not is_point_below_line([0, 0], [1, 0], math.pi / 2)
    assert is_point_below_line([0, 0], [1, 0], -math.pi / 2)

    assert not is_point_inside_rectangle([0.0725, 0.0649], [0.0552, 0.0244], 0.054, 0.0856, -1.2238944529161517)
    assert is_point_inside_rectangle([0.0619, 0.0634], [0.0552, 0.0244], 0.054, 0.0856, -1.2238944529161517)
    assert not is_point_inside_rectangle([0.0313, -0.0195], [0.0552, 0.0244], 0.054, 0.0856, -1.2238944529161517)
    assert not is_point_inside_rectangle([0.0538, 0.0726], [0.0552, 0.0244], 0.054, 0.0856, -1.2238944529161517)
