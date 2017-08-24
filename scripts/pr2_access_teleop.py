#! /usr/bin/env python

import rospy
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX
import camera_info_messages
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

camera_info_mapping = {'camera1': camera_info_messages.camera1, 'camera2': camera_info_messages.camera2}
transform_broadcaster_mapping = {'camera1': ((0.5, 0, 3), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
                                'camera2': ((0.3, -1.5, 0.5), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link')}
move_group_name = "right_arm"

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def publish_camera_transforms():
    for key in transform_broadcaster_mapping:
        tb = TransformBroadcaster()
        transform_data = transform_broadcaster_mapping[key]
        tb.sendTransform(transform_data[0], transform_data[1], transform_data[2], transform_data[3], transform_data[4])


def publish_camera_info():
    for key in camera_info_mapping:
        pub = rospy.Publisher(key + '/camera_info', camera_info_messages.CameraInfo, queue_size=10)
        pub.publish(camera_info_mapping[key])


def dpx_to_distance(dx, dy, camera_name, current_ps, offset):
    print("The dx is " + str(dx) + " the dy is " + str(dy) + " and the camera name is " + camera_name)

    big_z_mappings = {'camera1': transform_broadcaster_mapping['camera1'][0][2] - current_ps.pose.position.z,
                      'camera2': transform_broadcaster_mapping['camera2'][0][1] - current_ps.pose.position.y}

    print("The frame_id for the current pose is " + current_ps.header.frame_id)
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info_mapping[camera_name])
    x, y, z = camera_model.projectPixelTo3dRay((dx, dy))
    print " x : {} , y : {} , z : {}".format(x, y, z)
    x_center, y_center, z_center = camera_model.projectPixelTo3dRay((0, 0))
    big_z = abs(big_z_mappings[camera_name])
    print("The big_z for " + camera_name + " is " + str(big_z))
    big_x = (x / z) * big_z  # Use law of similar trianges to solve
    big_y = (y / z) * big_z

    big_x_center = (x_center / z_center) * big_z
    big_y_center = (y_center / z_center) * big_z

    print("The x_center  is " + str(x_center) + " the y_center  is " + str(y_center) + " and the z_center is " + str(
        z_center))
    print(
    "The x distance is " + str(big_x - big_x_center) + " the y distance is " + str(big_y - big_y_center) + " and the camera name is " + camera_name + "\n")
    if offset:
        return big_x - big_x_center, big_y - big_y_center
    else:
        return big_x, big_y


def modified_stamped_pose(x_distance, y_distance, camera_name, original_pose_stamped):
    modified_ps = original_pose_stamped
    if camera_name == 'camera1':
        original_pose_stamped.pose.position.x += x_distance  # These directions came from looking at the cameras in rviz
        original_pose_stamped.pose.position.y -= y_distance
    elif camera_name == 'camera2':
        original_pose_stamped.pose.position.x += x_distance
        original_pose_stamped.pose.position.z -= y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return modified_ps


def add_marker(x_distance, y_distance, ps, camera_name, self):
    controls = InteractiveMarkerControl()

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    int_marker = InteractiveMarker()
    int_marker.header = ps.header
    int_marker.name = "click_location"

    if camera_name == 'camera1':
        int_marker.pose.position.z = ps.pose.position.z
        int_marker.pose.position.x = transform_broadcaster_mapping['camera1'][0][0] + x_distance
        int_marker.pose.position.y = transform_broadcaster_mapping['camera1'][0][1] - y_distance
    elif camera_name == 'camera2':
        int_marker.pose.position.y = ps.pose.position.y
        int_marker.pose.position.x = transform_broadcaster_mapping['camera2'][0][0] + x_distance
        int_marker.pose.position.z = transform_broadcaster_mapping['camera2'][0][2] - y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')

    int_marker.pose.orientation.w = 1
    controls.markers.append(box_marker)
    int_marker.controls.append(controls)
    self._im_server.insert(int_marker)
    self._im_server.applyChanges()
    print("Changes should now be applied")


class MoveByDelta(object):
    def __init__(self, move_group):
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/delta', DeltaPX, self.callback)

    def callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.delta_x, data.delta_y, data.camera_name, ps, True)
        ps2 = modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        self._move_group.set_pose_target(ps2)
        self._move_group.go(wait=False)


class MoveByAbsolute(object):
    def __init__(self, move_group):
        self._move_group = move_group
        self._im_server = InteractiveMarkerServer('im_server', q_size=10)

    def start(self):
        rospy.Subscriber('/access_teleop/absolute', PX, self.absolute_callback)

    def absolute_callback(self, data):
        print("We got the pixel with x of " + str(data.pixel_x) + " and y of " + str(data.pixel_y))
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        add_marker(x_distance, y_distance, ps, data.camera_name, self)


def main():
    rospy.init_node('access_gripper_teleop')
    wait_for_time()

    print("Don't forget to launch the move group server with roslaunch pr2_moveit_config move_group.launch")
    move_group = MoveGroupCommander(move_group_name)

    move_by_delta = MoveByDelta(move_group)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(move_group)
    move_by_absolute.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_camera_transforms()
        publish_camera_info()
        rate.sleep()


if __name__ == "__main__":
    main()