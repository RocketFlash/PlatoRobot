#!/usr/bin/env python
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
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


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


def callback_holo(data):
    global sub_hololens, pointcloud_holo, done_holo
    x_points, y_points, z_points = [], [], []
    rospy.loginfo("Data from hololens received!")
    for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
        x_points.append(point[0])
        y_points.append(point[1])
        z_points.append(point[2])
    pointcloud_holo = np.column_stack((x_points, y_points, z_points))
    done_holo = True
    sub_holo.unregister()
    rospy.loginfo('Holo pointcloud was detected!')


def callback_plato(data):
    global sub_plato, pointcloud_plato, pointcloud_holo, done_plato, done_holo
    global threshold, number_of_initials, angle_from, angle_to
    global max_iterations, tolerance, tf_buffer, tf_listener, number_of_points_icp

    if done_holo:
        transform1 = None
        try:
            rospy.loginfo("Start getting data from velodyne")
            transform1 = tf_buffer.lookup_transform("map",
                                                    data.header.frame_id,
                                                    rospy.Time(0))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return

        data = do_transform_cloud(data, transform1)
        done_holo = False
        x_points, y_points, z_points = [], [], []
        for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
            x_points.append(point[0])
            y_points.append(point[1])
            z_points.append(point[2])
        pointcloud_plato = np.column_stack((x_points, y_points, z_points))
        sub_plato.unregister()

        rospy.loginfo("Data from plato received!")
        rospy.loginfo("Matching...")
        T = match_pointclouds(pointcloud_holo, pointcloud_plato, threshold,
                              number_of_initials, angle_from, angle_to,
                              max_iterations, tolerance, number_of_points_icp)

        broadcast_tf(T)
        rospy.loginfo('Matching done!')


def match_pointclouds(xyz_hololens, xyz_plato,
                      threshold=4.5, number_of_initials=5,
                      angle_from=-20, angle_to=20,
                      max_iterations=100, tolerance=1e-8,
                      number_of_points_icp=5000):
    global list_of_T, xyz_h, xyz_p, num_prs
    # cut all points < threshold
    xyz_plato = xyz_plato[xyz_plato[:, 0] < threshold, :]
    xyz_plato = xyz_plato[xyz_plato[:, 0] > -threshold, :]
    xyz_plato = xyz_plato[xyz_plato[:, 1] < threshold, :]
    xyz_plato = xyz_plato[xyz_plato[:, 1] > -threshold, :]

    rospy.loginfo('Number of points in hololens cloud  : {}'.format(
        xyz_hololens.shape[0]))
    rospy.loginfo('Number of points in plato cloud     : {}'.format(
        xyz_plato.shape[0]))

    # Make same number of points in two poinclouds
    # size = min(pc1.shape[0], pc2.shape[0])
    num_of_el = 0
    if xyz_hololens.shape[0] > xyz_plato.shape[0]:
        num_of_el = xyz_plato.shape[0]
        xyz_hololens = xyz_hololens[np.random.choice(
            xyz_hololens.shape[0], num_of_el, replace=False)]
    else:
        num_of_el = xyz_hololens.shape[0]
        xyz_plato = xyz_plato[np.random.choice(
            xyz_plato.shape[0], num_of_el, replace=False)]

    if num_of_el > number_of_points_icp:
        pass
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
                              max_iterations=max_iterations,
                              tolerance=tolerance)
        if np.sum(distances) < min_sum_distances:
            min_sum_distances = np.sum(distances)
            T_best = T.dot(T_trans)
        rospy.loginfo('===================================')
        rospy.loginfo('Initial angle         : {}'.format(
            (180 / math.pi) * current_angle))
        rospy.loginfo('Number of iterations  : {}'.format(i))
        rospy.loginfo('Total distances       : {}'.format(
            np.round(np.sum(distances), 3)))
        rospy.loginfo('Transformation matrix : \n {}'.format(np.round(T, 3)))
        current_angle += angle_increment

    xyz_best = T_best.dot(xyz_to_transform.T)
    xyz_best = xyz_best[:3, :].T
    xyz_p = xyz_to_transform
    xyz_h = xyz_hololens
    number_of_points = num_prs
    scale, shear, angles, translate, perspective = decompose_matrix(T_best)
    angle1 = np.linspace(0, angles[0], number_of_points)
    angle2 = np.linspace(0, angles[1], number_of_points)
    angle3 = np.linspace(0, angles[2], number_of_points)
    translate_x = np.linspace(0, translate[0], number_of_points)
    translate_y = np.linspace(0, translate[1], number_of_points)
    translate_z = np.linspace(0, translate[2], number_of_points)

    for i in range(number_of_points):
        curr_angles = np.array([angle1[i], angle2[i], angle3[i]])
        curr_translate = np.array(
            [translate_x[i], translate_y[i], translate_z[i]])
        curr_T = compose_matrix(scale=None, shear=None, angles=curr_angles, translate=curr_translate,
                                perspective=None)
        list_of_T.append(curr_T)

    return T_best


def broadcast_tf(T):
    global frame_holo, frame_plato, xyz_h, xyz_p, num_prs
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()
    scale, shear, rpy_angles, translation_vector, perspective = \
        tf.transformations.decompose_matrix(T)
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = frame_holo
    static_transformStamped.child_frame_id = frame_plato

    static_transformStamped.transform.translation.x = float(
        translation_vector[0])
    static_transformStamped.transform.translation.y = float(
        translation_vector[1])
    static_transformStamped.transform.translation.z = float(
        translation_vector[2])

    quat = tf.transformations.quaternion_from_euler(
        float(rpy_angles[0]), float(rpy_angles[1]), float(rpy_angles[2]))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

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


def listener():
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


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


#################### PARAMETERS ###########################
rospy.init_node('match_point_clouds', log_level=rospy.INFO)
# topic_name_holo = 'holo/point_cloud2'
topic_name_holo = 'holo/point_cloud2'
topic_name_plato = 'velodyne_points'
frame_holo = 'holo'
frame_plato = 'map'
sub_holo = rospy.Subscriber(
    topic_name_holo, PointCloud2, callback_holo, queue_size=1)
sub_plato = rospy.Subscriber(
    topic_name_plato, PointCloud2, callback_plato, queue_size=1)
hololens_pub = rospy.Publisher("points_holo", PointCloud2, queue_size=1)
pointcloud_holo = []
pointcloud_plato = []
done_holo = False
done_plato = False
threshold = 4.5
number_of_initials = 10
angle_from = -180
angle_to = 180
max_iterations = 500
tolerance = 1e-10
number_of_points_icp = 5000
list_of_T = []
xyz_h = 0
xyz_p = 0
num_prs = 20
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


listener()
