import time
import math
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import sys

from std_srvs.srv import Empty


from tf2_ros import Duration

# Global variables
UV_MIN_LEVEL = 40  #10*[mJ]  (valore riscalato)


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.localization_complete = False


        # Initialize the robot's pose variables
        self.amcl_pose = PoseWithCovarianceStamped()
        # Variable for the energy map
        self.energy_matrix = OccupancyGrid()
        self.pub_time = None
        self.energy_matrix_received = False
        self.localization_complete = False

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = False
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                        'amcl_pose',
                                                        self._amclPoseCallback,
                                                        qos_profile)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                    'initialpose',
                                                    10)
        
        self.ok_pub = self.create_publisher(Bool, '/ok_topic', qos_profile)
    

        self.declare_parameter('x_pose',  1.5)  
        self.declare_parameter('y_pose',  3.5) 
        self.declare_parameter('global_amcl_flag',  1)

        self.x_pose = float(self.get_parameter('x_pose').value )
        self.y_pose = float(self.get_parameter('y_pose').value )
        self.global_amcl_flag = int(self.get_parameter('global_amcl_flag').value )


        self.client = self.create_client(Empty, 'reinitialize_global_localization')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Il servizio non è ancora disponibile, in attesa...')
        self.req = Empty.Request()

        self.initial_pose_sent = False

    def send_request(self):
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)


    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Richiesta di re-inizializzazione globale inviata!')
        except Exception as e:
            self.get_logger().error('Servizio di re-inizializzazione fallito: %r' % (e,))




    def publish_vel(self, stop_rotate):
        # Create Twist Message
        msg = Twist()

        # Assign Random velocity to Twist Message
        msg.linear.x = 0.0
        if not stop_rotate:
            msg.angular.z = 0.3

        else:
            msg.angular.z = 0.0

      
        # Publish Twist Message
        self.publisher.publish(msg)
      




    def goal_generator (self, dist):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.position.x = self.amcl_pose.pose.pose.position.x + dist
        goal_pose.pose.position.y = self.amcl_pose.pose.pose.position.y
        return goal_pose.pose.position.x, goal_pose.pose.position.y, self.localization_complete

                         


    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                    self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        #self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        if self.initial_pose_received:
            self._setInitialPose()
            self.info('amcl_pose received')
            self.get_logger().info(f"Initial pose received: {self.amcl_pose.pose.pose.position.x} {self.amcl_pose.pose.pose.position.y}")
        
        rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.amcl_pose.pose.pose.position.x = msg.pose.pose.position.x
        self.amcl_pose.pose.pose.position.y = msg.pose.pose.position.y
        self.amcl_pose.header.frame_id = msg.header.frame_id
        self.amcl_pose.pose.covariance = msg.pose.covariance

        self.amcl_pose.pose.pose.orientation = msg.pose.pose.orientation
        self.initial_pose_received = True

        covariance = list(self.amcl_pose.pose.covariance)
        diagonal_elements = [covariance[i*6+i] for i in range(6)]
        threshold =  0.1  # Example threshold, adjust as needed
        if all(element < threshold for element in diagonal_elements):
            self.localization_complete = True
        else:
            self.localization_complete = False
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = self.x_pose
        msg.pose.pose.position.y = self.y_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        self.initial_pose_sent = True
        return
    

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return



def main(argv=sys.argv[1:]):
    time.sleep(5)

    
    
    rclpy.init()
    navigator = BasicNavigator()

    navigator.get_logger().info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    # Waiting for amcl_pose to be received
    navigator.info('Waiting for amcl_pose to be received and for the navigation2 stack to become active...')
    # Wait for navigation to fully activate
    #navigator.waitUntilNav2Active()
    #input('Navigation2 ok, enter to continue')


    if navigator.global_amcl_flag == 1:

        navigator.send_request()

        navigator.get_logger().info("GLOBAL AMCL: RANDOM NAVIGATION: ")
        # Rotate for a certain time
        rotation_duration = Duration(seconds=30.0)  # Example duration, adjust as needed
        
        while not navigator.localization_complete:

            start_time = navigator.get_clock().now()
            while navigator.get_clock().now() - start_time < rotation_duration:
                navigator.publish_vel(0)
                time.sleep(0.01)
                
            navigator.publish_vel(1)
            time.sleep(0.1)

            # Navigate to a goal
            if not navigator.localization_complete:
                
                goal_pose_x, goal_pose_y, navigator.localization_complete = navigator.goal_generator(0.55)
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.orientation.z = 0.0
                goal_pose.pose.orientation.w = 1.0
                goal_pose.pose.position.x = goal_pose_x
                goal_pose.pose.position.y = goal_pose_y



                navigator.goToPose(goal_pose)
                navigator.get_logger().info('Navigator go to pose')
                
                while not navigator.isNavComplete():
                    navigator.get_logger().info("Waiting for goal reaching...")
                    time.sleep(0.1)
                    
                        
            navigator.get_logger().info('Goal concluded')
    


    else:
        while not navigator.initial_pose_sent:
            navigator.get_logger().info("SENDING THE INITIAL POSE: ")
            navigator.setInitialPose(None)
            time.sleep(0.1)
        pass

    navigator.get_logger().info("Localization complete: Low variance")

    ok_msg = Bool()
    ok_msg.data = True
    navigator.ok_pub.publish(ok_msg)

    time.sleep(30)
    navigator.destroy_node()
    rclpy.shutdown()

    exit(0)

