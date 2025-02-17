#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Trigger
from ebot_docking.srv import DockSw
import math, statistics
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from linkattacher_msgs.srv import AttachLink, DetachLink


class EbotOdomDockController(Node):

    def __init__(self):
        super().__init__('ebot_odom_dock_controller')
        self.callback_group = ReentrantCallbackGroup()

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
        self.start_dock_srv = self.create_service(DockSw, 'start_docking', self.start_docking_callback, callback_group=self.callback_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # self.link_attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        # while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Link attacher service not available, waiting again...')

        # self.link_attach_req = AttachLink.Request()


        self.start_docking = False  # Initially it will be False, to get trigger from outside


        self.is_controller_running_linear = False
        self.is_controller_running_angular = False
        self.dock_align=False

        self.ebot_pose = [0,0,0]

        self.usrright_value = None
        self.usrleft_value = None
        self.kp_linear = 0.5
        self.kp_angular = 3
        self.odom_msg = None
        self.linear_tolerance = 0.02
        self.angular_tolerance = 0.02


        self.desired_x = 0.1
        self.desired_z = 0.0
        self.rack_no = None
        self.controller_timer = self.create_timer(0.1, self.controller_loop)


        # self.is_controller_running_angular

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

        # print("RIGHT", self.usrright_value)

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

        # print("LEFT", self.usrleft_value)
        

    def odometry_callback(self, msg):
        self.odom_msg = msg

        self.ebot_pose[0] = msg.pose.pose.position.x
        self.ebot_pose[1] = msg.pose.pose.position.y

        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _,_, yaw = euler_from_quaternion(orientation_list)

        self.ebot_pose[2] = yaw


    def normalize_angle(self,angle):

        ''' Converting angle to the shortest angle possible '''
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def controller_loop(self):

        # self.get_logger().info("Controller_LOOP running")
        
        if self.start_docking:

            if self.is_controller_running_angular:

                error_z=self.normalize_angle(self.desired_z-self.ebot_pose[2])

                # print("ANGLE CORRECTION",self.ebot_pose[2] , error_z, self.is_controller_running_angular, self.is_controller_running_linear)

                twist_msg = Twist()

                if round(abs(error_z), 3) > self.angular_tolerance:
                    
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = self.kp_angular*error_z

                    self.cmd_vel_publisher.publish(twist_msg)
                else:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0

                    self.cmd_vel_publisher.publish(twist_msg)

                    self.get_logger().info("Oriented towards goal")
                    self.is_controller_running_angular = False
                    self.is_controller_running_linear = True
                    
            if self.is_controller_running_linear:

                twist_msg = Twist()

                # print(self.usrleft_value, self.usrright_value)

                distance_from_rack = statistics.mean((self.usrleft_value, self.usrright_value))
                error_x = distance_from_rack - self.desired_x

                linear_velocity = - self.kp_linear * error_x
                angular_velocity = 0.0

                # print("LINEAR CORRECTION",distance_from_rack, error_x)

                if abs(error_x) > self.linear_tolerance:
                    twist_msg.linear.x = linear_velocity
                    twist_msg.angular.z = angular_velocity

                    self.cmd_vel_publisher.publish(twist_msg)

                else:

                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0

                    self.cmd_vel_publisher.publish(twist_msg)

                    self.get_logger().info(f'Reached Goal Successfully !!!')

                    self.dock_align=True
                    self.is_controller_running_linear = False

                    # if self.rack_no != None:
                    #     self.attach_with_ebot(self.rack_no)
                    # else:
                    #     self.get_logger().error(" Rack_NO not defined")


    def attach_with_ebot(self, rack, link='link'):

        self.get_logger().info(" Link Attach Request sending !!")

        self.link_attach_req.model1_name = 'ebot'
        self.link_attach_req.link1_name = 'base_link'
        self.link_attach_req.model2_name = rack
        self.link_attach_req.link2_name = link

        future_link=self.link_attach_cli.call_async(self.link_attach_req)
        rclpy.spin_until_future_complete(self, future_link)

        print("Linking response: ", future_link.result())

        self.get_logger().info(" Link Attach Request send !!")


    def start_docking_callback(self, request, response):

        self.dock_align=False
        self.start_docking = True
        self.is_controller_running_angular = True

        self.desired_z = request.orientation
        self.rack_no = request.rack_no

        self.get_logger().info(" DOCKING STARTED !!")

        rate = self.create_rate(2, self.get_clock())

        while(not self.dock_align):
            self.get_logger().info("Waiting until it is aligned !")
            rate.sleep()

        # self.get_logger().info("Status of dock", self.dock_align)

        response.success = True
        response.message = "Controller_started"
        return response
    

def main(args=None):
    rclpy.init(args=args)

    ebot_odom_dock_controller = EbotOdomDockController()

    executor= MultiThreadedExecutor()
    executor.add_node(ebot_odom_dock_controller)
    
    executor.spin()
    # rclpy.spin(ebot_odom_dock_controller)

    ebot_odom_dock_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
