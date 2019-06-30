#!/usr/bin/env python
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import numpy as np
import rosbag
from icp import icp
import vispy.scene
from vispy.scene import visuals
import math
import rospy


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


def xyz_array_from_bag(bagfilename, topic_name, index=0):
    """
    Function reads bag file and returns numpy array of pointcloud
    xyz point coordinates
    Input:
        bagfilename: full path to the bag file
        topic_name: PointCloud2 topic name
        index: index of cloud frame in bag file
    Output:
        xyz_array: array of xyz points of PointCloud2
    """
    bag = rosbag.Bag(bagfilename)
    i = 0
    for topic, msg, t in bag.read_messages(topics=['/' + topic_name]):
        if i == index:
            x_points, y_points, z_points = [], [], []
            for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
                x_points.append(point[0])
                y_points.append(point[1])
                z_points.append(point[2])
            xyz_array = np.column_stack((x_points, y_points, z_points))
            break
        elif i > index:
            break
        else:
            i += 1
    bag.close()
    return xyz_array


def plot_3d_point_clouds(pc1, pc2):
    """
    Function plots 3d points from two pointclouds
    Input:
        pc1: Nx3 numpy array of first pointcloud
        pc2: Nx3 numpy array of second pointcloud
    """
    canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
    view = canvas.central_widget.add_view()
    scatter1 = visuals.Markers()
    scatter1.set_data(pc1, edge_color=(1, 1, 1, .5),
                      face_color=(1, 1, 1, .5), size=3)
    scatter2 = visuals.Markers()
    scatter2.set_data(pc2, edge_color=(1, 1, 1, .5),
                      face_color=(1, 0, 0, .5), size=5)

    view.add(scatter1)
    view.add(scatter2)

    view.camera = 'turntable'  # or try 'arcball'

    # add a colored 3D axis for orientation
    axis = visuals.XYZAxis(parent=view.scene)

    vispy.app.run()


#################### PARAMETERS ###########################
bag_files_path = '../../plato_navigation/bagfiles/'
bagfile_name_hololens = 'hololens.bag'
bagfile_name_plato = 'plato.bag'
topic_name_hololens = 'pointCloud2Test'
topic_name_plato = 'velodyne_points'
index_hololens = 1
index_plato = 50
# cut all points < threshold
threshold = 4.5
number_of_initials = 5
angle_from = -20
angle_to = 20

xyz_hololens = xyz_array_from_bag('{}{}'.format(bag_files_path,
                                                bagfile_name_hololens),
                                  topic_name=topic_name_hololens,
                                  index=index_hololens)
xyz_plato = xyz_array_from_bag('{}{}'.format(bag_files_path,
                                             bagfile_name_plato),
                               topic_name=topic_name_plato,
                               index=index_plato)
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
for rot_i in range(number_of_initials):
    T_trans = rotation_matrix([0, 0, 1], current_angle)
    xyz_new = T_trans.dot(xyz_to_transform.T)
    xyz_new = xyz_new[:3, :].T
    T, distances, i = icp(xyz_new, xyz_hololens, init_pose=None,
                          max_iterations=1000, tolerance=1e-15)
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


plot_3d_point_clouds(xyz_hololens, xyz_best)
