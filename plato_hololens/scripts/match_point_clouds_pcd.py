#!/usr/bin/env python
import sensor_msgs.point_cloud2
import numpy as np
from open3d import *
import pypcd
from icp import icp
import vispy.scene
from vispy.scene import visuals
from vispy import app
import math
import rospy
from transformations import *
from moviepy.editor import VideoClip
from vispy.gloo.util import _screenshot
import PIL
import mayavi.mlab as mlab

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import sensor_msgs.point_cloud2
import numpy as np
import tf
import tf2_ros
from icp import icp
import math
import rospy
import tf2_py as tf2


def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg


def rotation_matrix(axis, theta):
    """
    Function calculates homogeneous matrix based on
    rotation around vector 'axis' on angle theta based on
    Euler-Rodrigues formula:
    (https://en.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula)
    Input:
        axis: [x,y,z] vector of rotation axe
        theta: angle to rotate
    Output:
        T:  final homogeneous transformation that rotates points
            around 'axis' vector on theta angle
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d

    R = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                  [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                  [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    T = np.identity(4)
    T[:3, :3] = R
    return T


def xyz_array_from_pcd(pcd_path):
    """
    Function reads pcd file and returns np array of pointcloud
    xyz point coordinates
    Input:
        pcd_path: full path to the pcd file
    Output:
        xyz_array: array of xyz points of PointCloud2
    """

    pcd_load = read_point_cloud(pcd_path)
    # convert Open3D.PointCloud to np array
    xyz_array = np.asarray(pcd_load.points)
    return xyz_array


#################### PARAMETERS ###########################
_EPS = np.finfo(float).eps * 4.0
pcd_files_path = '../data/'
pcd_name_hololens = 'hololens.pcd'
pcd_name_plato = 'plato.pcd'
# cut all points < threshold
threshold = 4
number_of_initials = 1
angle_from = -180
angle_to = 180
max_iterations = 1000
tolerance = 1e-9
num_prs = 20

xyz_hololens = xyz_array_from_pcd(
    '{}{}'.format(pcd_files_path, pcd_name_hololens))
xyz_plato = xyz_array_from_pcd(
    '{}{}'.format(pcd_files_path, pcd_name_plato))
xyz_plato = xyz_plato[xyz_plato[:, 0] < threshold, :]
xyz_plato = xyz_plato[xyz_plato[:, 0] > -threshold, :]
xyz_plato = xyz_plato[xyz_plato[:, 1] < threshold, :]
xyz_plato = xyz_plato[xyz_plato[:, 1] > -threshold, :]


print('Number of points in hololens cloud  : {}'.format(xyz_hololens.shape[0]))
print('Number of points in plato cloud     : {}'.format(xyz_plato.shape[0]))

# Make same number of points in two poinclouds
# size = min(pc1.shape[0], pc2.shape[0])
if xyz_hololens.shape[0] > xyz_plato.shape[0]:
    num_of_el = xyz_plato.shape[0]
    xyz_hololens = xyz_hololens[np.random.choice(
        xyz_hololens.shape[0], num_of_el, replace=False)]
else:
    num_of_el = xyz_hololens.shape[0]
    xyz_plato = xyz_plato[np.random.choice(
        xyz_plato.shape[0], num_of_el, replace=False)]


xyz_to_transform = np.c_[xyz_plato, np.ones(xyz_plato.shape[0])]

current_angle = angle_from * (math.pi / 180)
angle_increment = (angle_to - angle_from) * \
    (math.pi / 180) / number_of_initials
min_sum_distances = 10000000000
T_best = np.eye(4)

xyz_p = xyz_to_transform
xyz_h = xyz_hololens

for rot_i in range(number_of_initials):
    T_trans = rotation_matrix([0, 0, 1], current_angle)
    xyz_new = T_trans.dot(xyz_to_transform.T)
    xyz_new = xyz_new[:3, :].T
    T, distances, i = icp(xyz_new, xyz_hololens, init_pose=None,
                          max_iterations=max_iterations, tolerance=tolerance)

    xyz_current = T.dot(xyz_to_transform.T)
    xyz_current = xyz_current[:3, :].T

    if np.sum(distances) < min_sum_distances:
        min_sum_distances = np.sum(distances)
        T_best = T.dot(T_trans)
    print('===================================')
    print('Initial angle         : {}'.format((180 / math.pi) * current_angle))
    print('Number of iterations  : {}'.format(i))
    print('Total distances       : {}'.format(np.round(np.sum(distances), 3)))
    print('Transformation matrix : \n {}'.format(np.round(T, 3)))
    current_angle += angle_increment

xyz_best = T_best.dot(xyz_to_transform.T)
xyz_best = xyz_best[:3, :].T

number_of_points = 20
scale, shear, angles, translate, perspective = decompose_matrix(T_best)
angle1 = np.linspace(0, angles[0], number_of_points)
angle2 = np.linspace(0, angles[1], number_of_points)
angle3 = np.linspace(0, angles[2], number_of_points)
translate_x = np.linspace(0, translate[0], number_of_points)
translate_y = np.linspace(0, translate[1], number_of_points)
translate_z = np.linspace(0, translate[2], number_of_points)
list_of_T = []

for i in range(number_of_points):
    curr_angles = np.array([angle1[i], angle2[i], angle3[i]])
    curr_translate = np.array([translate_x[i], translate_y[i], translate_z[i]])
    curr_T = compose_matrix(scale=None, shear=None, angles=curr_angles, translate=curr_translate,
                            perspective=None)
    list_of_T.append(curr_T)

rospy.init_node('match_point_clouds', log_level=rospy.INFO)
pub_holo = rospy.Publisher('pc_holo', PointCloud2, queue_size=10)
pub_plato = rospy.Publisher('pc_plato', PointCloud2, queue_size=10)

i = 0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    xyz_curr = list_of_T[i].dot(xyz_p.T)
    xyz_curr = xyz_curr[:3, :].T
    msg_p = xyz_array_to_pointcloud2(
        xyz_curr, stamp=rospy.Time.now(), frame_id='map')
    msg_h = xyz_array_to_pointcloud2(
        xyz_h, stamp=rospy.Time.now(), frame_id='map')
    pub_holo.publish(msg_h)
    pub_plato.publish(msg_p)
    rate.sleep()
    i += 1
    if i == num_prs:
        i = 0
