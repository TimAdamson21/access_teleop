#! /usr/bin/env python

import rospy
import math
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX, PXAndTheta, Theta, TaskType, HeadZ
import fetch_api
import camera_info_messages
from std_msgs.msg import String, Header, ColorRGBA, Bool
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from shared_teleop_functions_and_vars import wait_for_time, quat_array_to_quat, publish_camera_transforms, publish_camera_info, \
    publish_gripper_pixels, dpx_to_distance, delta_modified_stamped_pose, \
    absolute_modified_stamped_pose, add_marker, addSetback, orientation_mapping, orientation_sign_mapping, camera_names
from gazebo_msgs.srv import GetModelState
import os
import rosbag
from sensor_msgs.msg import PointCloud2

HEAD_POSE = [1.7, -0.1, 0.25]  # the point in space where robot should look at
MODELS = {0: "cube_s", 1: "cube_m", 2: "cube_l", 3: "cube_xl", 4: "ball", 5: "stone"}  # models used in ARAT test
current_model_idx = 0  # index of the current model in MODELS

class MoveByDelta(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/delta', DeltaPX, self.callback, queue_size=1)

    def callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.delta_x, data.delta_y, data.camera_name, ps, True)
        ps2 = delta_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        pose_possible = self._arm.compute_ik(ps2, timeout=rospy.Duration(1))
        print(pose_possible)
        if pose_possible:  # This check will prevent some edge poses, but will also save time
            error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
            if error is not None:
                rospy.logerr(error)
            else:
                print("We got there!")


class MoveByAbsolute(object):

    def __init__(self, arm, move_group, status_pub):
        self._arm = arm
        self._move_group = move_group
        self._im_server = InteractiveMarkerServer('im_server', q_size=2)
        self._status_pub = status_pub

    def start(self):
        rospy.Subscriber('/access_teleop/absolute', PX, self.absolute_callback, queue_size=1)

    def absolute_callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        # add_marker(x_distance, y_distance, ps, data.camera_name, self)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        pose_possible = self._arm.compute_ik(ps2, timeout=rospy.Duration(1))
        print(pose_possible)
        if pose_possible:  # This check will prevent some edge poses, but will also save time
            self._status_pub.publish("moving")
            error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
            if error is not None:
                rospy.logerr(error)
            else:
                self._status_pub.publish("arrived")
        else:
            self._status_pub.publish("unreachable")


class MoveAndOrient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/move_and_orient', PXAndTheta, self.move_and_orient_callback, queue_size=1)

    def move_and_orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        # rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] = data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class MoveAndOrient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group
        self.SETBACK = 0.15

    def start(self):
        rospy.Subscriber('/access_teleop/move_and_orient', PXAndTheta, self.move_and_orient_callback, queue_size=1)

    def move_and_orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        # rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] = data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        ps3 = addSetback(self.SETBACK, data.theta, data.camera_name, ps2)
        error = self._arm.move_to_pose(ps3, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class Orient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/orient', Theta, self.orient_callback, queue_size=1)

    def orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        # rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] += data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        error = self._arm.move_to_pose(ps, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class WristRoll(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/wrist_roll', Theta, self.wrist_roll_callback, queue_size=1)

    def wrist_roll_callback(self, data):
        self._move_group.clear_pose_targets()
        arm_values = self._move_group.get_current_joint_values()
        arm_values[6] += data.theta
        self._move_group.set_joint_value_target(arm_values)
        self._move_group.go()


# Added by Xinyi for switching the task in action test
class BaseSwitchTask(object):
    def __init__(self, base):
        self._base = base

    def start(self):
        rospy.Subscriber('/access_teleop/task_type', TaskType, self.base_switch_task_callback, queue_size=1)

    def base_switch_task_callback(self, data):
        # delete the current test object and spawn the specified test object
        os.system("$(rospack find access_teleop)/scripts/switch_object.sh " + str(MODELS[data.delete_type]) + " " + str(MODELS[data.add_type]))
        
        # record the current model index
        global current_model_idx
        current_model_idx = data.add_type

        #### below is the code for moving Fetch from one table to another
        # if data.task_type is 0:  # go to previous task
        #     self._base.turn(math.pi / 2)
        #     self._base.align_with_y_axis_pos()

        #     self._base.go_forward(0.95, 0.3)
        #     self._base.stop()
        #     rospy.sleep(0.03)

        #     self._base.turn(-math.pi / 2)
        #     self._base.align_with_x_axis_pos()
        # else:  # go to next task
        #     self._base.turn(-math.pi / 2)
        #     self._base.align_with_y_axis_neg()

        #     self._base.go_forward(0.95, 0.3)
        #     self._base.stop()
        #     rospy.sleep(0.03)

        #     self._base.turn(math.pi / 2)
        #     self._base.align_with_x_axis_pos()


# Added by Xinyi for tilting robot's head
class HeadTilt(object):
    def __init__(self, head):
        self._head = head

    def start(self):
        rospy.Subscriber('/access_teleop/head_z', HeadZ, self.head_tilt_callback, queue_size=1)

    def head_tilt_callback(self, data):
        HEAD_POSE[2] += data.tilt
        self._head.look_at("base_link", HEAD_POSE[0], HEAD_POSE[1], HEAD_POSE[2])



def main():
    rospy.init_node('gripper_teleop')
    wait_for_time()

    # Added by Xinyi:
    # Initialize Settings for the Test (Part 1)

    # Raise the torso to allow arm movement
    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)

    # Set arm joints
    # Order: body ---> gripper
    # b: blue joint
    # g: gray joint
    # order: [b, g, b, g, b, g, b]
    # OPTION 1: Follow given trajectory
    arm = fetch_api.Arm()
    arm_initial_poses = [1.0, 1.25, 1.0, -2.25, -0.3, 1.0, 0.0]
    arm.move_to_joints(fetch_api.ArmJoints.from_list(arm_initial_poses))
    # for recording bag file
    # arm_viz_poses = [1.0, 1.25, 1.0, -2.25, 2.25, 2.25, 0.0]
    # arm.move_to_joints(fetch_api.ArmJoints.from_list(arm_viz_poses))

    # OPTION 2: Use motion planning
    # INITIAL_POSES = [
    #         ("shoulder_pan_joint", 1.0), ("shoulder_lift_joint", 1.2), ("upperarm_roll_joint", 1.0), ("elbow_flex_joint", -2.0), 
    #         ("forearm_roll_joint", -0.3), ("wrist_flex_joint", 1.2), ("wrist_roll_joint", 0.0)]
    # arm.move_to_joint_goal(INITIAL_POSES, replan=True)

    # INITIAL_POSES = PoseStamped()
    # INITIAL_POSES.header.frame_id = 'base_link'
    # INITIAL_POSES.pose.position.x = 0.6
    # INITIAL_POSES.pose.position.y = -0.1
    # INITIAL_POSES.pose.position.z = 0.3
    # INITIAL_POSES.pose.orientation.w = 1
    # error = arm.move_to_pose(INITIAL_POSES, replan=True)
    # if error is not None:
    #     arm.cancel_all_goals()
    #     rospy.logerr('FAIL TO MOVE TO INITIAL_POSES {}'.format(error))
    # else:
    #     rospy.loginfo('MOVED TO INITIAL_POSES')


    # Set the base position
    base = fetch_api.Base()
    # OPTION 1: Start from origin
    # move Fetch from the origin to the table
    # base.go_forward(1.6, 0.5)  # value (distance in meters)
    # base.turn(math.pi / 3)     # value (angle in degrees)
    # base.go_forward(3.3, 0.5)  # value (distance in meters)
    # base.turn(-math.pi / 3)

    # OPTION 2: Start right in front of the table
    base.align_with_x_axis_pos()
    base.go_forward(0.3, 0.5)


    # Avoid occlusion
    # OPTION 1: Freeze point cloud
    # freeze_pub = rospy.Publisher('/access_teleop/freeze_cloud', Bool, queue_size=5)
    # rospy.sleep(0.5)
    # publish freeze point cloud
    # freeze_pub.publish(Bool(data=True))

    # OPTION 2: Record a bag file of the surrounding
    head = fetch_api.Head()

    # tilt the head from -0.35 to 0.85
    # for i in range(6):
    #     head.look_at("base_link", HEAD_POSE[0], HEAD_POSE[1], -0.35 + i * 0.2)
    #     os.system("rosrun perception save_cloud world $(rospack find access_teleop)/bags/")
    # rospy.set_param("bag_file_refreshed", "true")
    # (end)

    move_group = MoveGroupCommander("arm")

    status_publisher = rospy.Publisher('/access_teleop/arm_status', String, queue_size=1)
    gripper_publisher = rospy.Publisher('/access_teleop/gripper_pixels', PX, queue_size=1)

    info_pubs = []
    for camera_name in camera_names:
        info_pubs.append([camera_name,
                          rospy.Publisher(camera_name + '/camera_info', camera_info_messages.CameraInfo, queue_size=1)])

    # Added by Xinyi
    # Debug: visualize camera positions
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    # (end)

    tb = TransformBroadcaster()

    camera_model = PinholeCameraModel()

    move_by_delta = MoveByDelta(arm, move_group)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(arm, move_group, status_publisher)
    move_by_absolute.start()

    move_and_orient = MoveAndOrient(arm, move_group)
    move_and_orient.start()

    orient = Orient(arm, move_group)
    orient.start()

    # Added by Xinyi
    wrist_roll = WristRoll(arm, move_group)
    wrist_roll.start()

    base_switch_task = BaseSwitchTask(base)
    base_switch_task.start()

    head_tilt = HeadTilt(head)
    head_tilt.start()

    # Initialize Settings for the Test (Part 2)
    # Add the first test object to Gazebo
    os.system("$(rospack find access_teleop)/scripts/switch_object.sh " + "NONE" + " " + str(MODELS[0]))
    
    # Move arm joints to positions for test
    arm_test_poses = [1.0, 1.25, 1.0, -2.25, -0.3, 1.0, 0.0]
    arm.move_to_joints(fetch_api.ArmJoints.from_list(arm_test_poses))

    # Adjust the head to look at the table
    # OPTION 1: Tilt by angle
    # head.pan_tilt(0, math.pi / 2)

    # OPTION 2: Look at a point in space
    head.look_at("base_link", HEAD_POSE[0], HEAD_POSE[1], HEAD_POSE[2])

    rospy.sleep(0.5)
    # (end)

    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        publish_camera_transforms(tb, vis_pub)
        publish_camera_info(info_pubs)
        publish_gripper_pixels(camera_model, move_group, gripper_publisher)

        # publish freeze point cloud
        # freeze_pub.publish(Bool(data=True))

        # # get the current model position and publish a marker of current model position
        # model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        # model_pos = model_state(MODELS[current_model_idx], 'base_link')
        # if model_pos.success:
        #     marker = Marker(
        #             type=Marker.SPHERE,
        #             id=current_model_idx,
        #             pose=Pose(Point(model_pos.pose.position.x, model_pos.pose.position.y, model_pos.pose.position.z), 
        #                       Quaternion(model_pos.pose.orientation.x, model_pos.pose.orientation.y, model_pos.pose.orientation.z, model_pos.pose.orientation.w)),
        #             scale=Vector3(0.05, 0.05, 0.05),
        #             header=Header(frame_id='base_link'),
        #             color=ColorRGBA(1.0, 0.5, 1.0, 0.5))
        #     vis_pub.publish(marker)

        rate.sleep()


if __name__ == "__main__":
    main()
