#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from robotics_project.msg import PickUpPoseAction, PickUpPoseGoal

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
# from IPython import embed

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):

        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose').replace(' ', '').split(',')
        self.pick_pose_topic = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.aruco_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        # Subscribe to topics
        rospy.Subscriber(self.aruco_pose_topic, PoseStamped, self.callback_aruco_pose)

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.pick_pose_pub = rospy.Publisher(self.pick_pose_topic, PoseStamped, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Instantiate services
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
        self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

        # Miscellaneous
        self.aruco_pose = None

        # Init state machine
        self.success_state = -1
        self.error_state = -2

        self.state = 0
        rospy.sleep(3)
        self.check_states()


    def callback_aruco_pose(self, pose):
        self.aruco_pose = pose


    def check_states(self):

        while not rospy.is_shutdown() and self.state != self.success_state:

            # State 0: tuck the arm
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                fail_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(10.0))

                if fail_tucking:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = self.error_state
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.loginfo("%s: Arm tucked.", self.node_name)
                    self.state = 1

                rospy.sleep(1)

            # State1: look down
            if self.state == 1:
                try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        self.state = 2
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = self.error_state
                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e
                    self.state = self.error_state

                rospy.sleep(3)

            # State 2: pick the cube
            if self.state == 2:
                # Call the pick service
                rospy.loginfo("%s: Trying to pick the cube", self.node_name)
                request = SetBoolRequest()
                request.data = True
                answer = self.pick_srv(request)

                if answer.success:
                    rospy.loginfo("%s: Cube picked!", self.node_name)
                    self.state = 3
                else:
                    rospy.loginfo("%s: Cube in holidays...", self.node_name)
                    self.state = self.error_state

            # State 3: running away from the table
            if self.state == 3:
                move_msg = Twist()
                move_msg.linear.x = -1

                rate = rospy.Rate(10)
                cnt = 0
                rospy.loginfo("%s: Running away with the cube", self.node_name)
                while not rospy.is_shutdown() and cnt < 45:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 4
                rospy.sleep(1)

            # State 4: moving towards table
            if self.state == 4:
                move_msg = Twist()

                rate = rospy.Rate(10)
                cnt = 0
                while not rospy.is_shutdown() and cnt < 5:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                move_msg.angular.z = 1
                cnt = 0
                rospy.loginfo("%s: Realizing approach manoeuvre", self.node_name)
                while not rospy.is_shutdown() and cnt < 30:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 5
                rospy.sleep(1)

            # State 5: placing the cube
            if self.state == 5:
                rospy.loginfo("%s: Trying to place the cube", self.node_name)
                request = SetBoolRequest()
                request.data = True
                answer = self.place_srv(request)

                if answer.success:
                    rospy.loginfo("%s: Cube maybe placed", self.node_name)
                    self.state = 6
                else:
                    rospy.loginfo("%s: Don't want to let the cube go...", self.node_name)
                    self.state = self.error_state

            # State 6: looking for the cube on the table
            if self.state == 6:
                rospy.loginfo("%s: Looking for the cube on the table", self.node_name)
                self.aruco_pose = None

                rospy.sleep(2.0)
                if self.aruco_pose is None:
                    rospy.loginfo("%s: Cube not on the table!", self.node_name)
                    self.state = self.error_state
                else:
                    rospy.loginfo("%s: Cube on the table!", self.node_name)
                    self.state = self.success_state

            # Error handling
            if self.state == self.error_state:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
