#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Trigger
import math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class EbotOdomDockController(Node):

    def __init__(self):
        super().__init__('ebot_odom_dock_controller')

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.start_dock_srv = self.create_service(Trigger, 'start_docking', self.start_docking_callback)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.docking = True    # UNDOCKING: FALSE | DOCKING: TRUE
        self.trigger_rack = True   # COMMUNICATE TO UR5: TRUE | ELSE FALSE

        if not self.docking:
            self.get_logger().info(f'Initialize Undocking...')
            self.desired_x = 1.70
            self.desired_z = 3.14
            if self.trigger_rack:
                ##Trigger ARM
                pass
                # self.trigger_ur_arm = self.create_client(Trigger, 'rack_trigger')
                # while not self.trigger_ur_arm.wait_for_service(timeout_sec=1.0):
                #     self.get_logger().info('Trigger Service not available, waiting...')

        elif self.docking:
            self.get_logger().info(f'Initialize Docking...')
            self.desired_x = -0.2
            self.desired_z = 0.0


        # self.desired_z = 1.57

        self.is_controller_running_linear = False
        self.is_controller_running_angular = False
        # self.desired_x = -0.2 #1.8
        # self.desired_y = 0.0
        # self.desired_z = 0.0 #3.14
        self.ebot_x = 0.0
        self.ebot_y = 0.0
        self.ebot_z = 0.0
        self.kp_linear = 0.5
        self.kp_angular = 1.5
        self.odom_msg = None
        self.linear_tolerance = 0.04
        self.angular_tolerance = 0.04

        # self.controller_timer = self.create_timer(0.1, self.controller_loop)


    def odometry_callback(self, msg):
        self.odom_msg = msg
        # self.get_logger().info(f'\n\n\nI heard: {str(msg.pose)}')
        self.ebot_x = msg.pose.pose.position.x
        self.ebot_y = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.ebot_z = yaw

        print("ANGLE:", yaw, math.degrees(yaw), self.desired_z - self.ebot_z)
        # self.get_logger().info(f'\n\n\nODOM: X: {str(self.ebot_x)}\tY: {str(self.ebot_y)}\tZ: {str(self.ebot_z)}')
        # self.get_logger().info(f'YAW: {self.ebot_z}')

    def controller_loop(self):
        if self.is_controller_running_angular:

            error_z = self.desired_z - self.ebot_z
            print(self.ebot_z, error_z)
            twist_msg = Twist()
            if abs(error_z) > self.angular_tolerance:
                if self.ebot_z < 0:
                    twist_msg.angular.z = -self.kp_angular*error_z
                else:
                    twist_msg.angular.z = self.kp_angular*error_z
                twist_msg.linear.x = 0.0
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
            error_x = self.desired_x - self.ebot_x
            if error_x >= 0:
                linear_velocity = -self.kp_linear * error_x
            else:
                linear_velocity = self.kp_linear * error_x
            angular_velocity = 0.0
            print(self.ebot_x, error_x)

            if abs(error_x) < self.linear_tolerance:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.get_logger().info(f'Reached Goal Successfully !!!')

                if not self.docking and self.trigger_rack:
                    request = Trigger.Request()
                    future = self.trigger_ur_arm.call_async(request)
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Docking triggered successfully')
                        else:
                            self.get_logger().info('Docking trigger failed: %s' % response.message)
                    else:
                        self.get_logger().error('Service call failed %r' % future.exception())

                self.is_controller_running_linear = False
            else:
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity

            self.cmd_vel_publisher.publish(twist_msg)

    def start_docking_callback(self, request, response):
        # self.get_logger().info(f'\n\nX: {str(self.ebot_x)}\tY: {str(self.ebot_y)}\tZ: {str(self.ebot_z)}')
        self.is_controller_running_angular = True

        response.success = True
        response.message = "Controller started"
        return response
    

def main(args=None):
    rclpy.init(args=args)

    ebot_odom_dock_controller = EbotOdomDockController()

    rclpy.spin(ebot_odom_dock_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ebot_odom_dock_controller.destroy_node()
    # cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()







# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion
# from std_srvs.srv import Trigger
# import math
# from geometry_msgs.msg import Twist

# class EbotOdomDockController(Node):

#     def __init__(self):
#         super().__init__('ebot_odom_dock_controller')

#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
#         self.start_dock_srv = self.create_service(Trigger, 'start_docking', self.start_docking_callback)
#         self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

#         self.is_controller_running_linear = False
#         self.is_controller_running_angular = False
#         self.desired_x = -0.2 #1.75
#         # self.desired_y = 0.0
#         self.desired_z = 0.0 #3.14
#         self.ebot_x = 0.0
#         self.ebot_y = 0.0
#         self.ebot_z = 0.0
#         self.kp_linear = 0.5
#         self.kp_angular = 1.5
#         self.odom_msg = None
#         self.linear_tolerance = 0.04
#         self.angular_tolerance = 0.04

#         self.controller_timer = self.create_timer(0.1, self.controller_loop)


#     def odometry_callback(self, msg):
#         self.odom_msg = msg
#         # self.get_logger().info(f'\n\n\nI heard: {str(msg.pose)}')
#         self.ebot_x = msg.pose.pose.position.x
#         self.ebot_y = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#         self.ebot_z = yaw
#         # self.get_logger().info(f'\n\n\nODOM: X: {str(self.ebot_x)}\tY: {str(self.ebot_y)}\tZ: {str(self.ebot_z)}')
#         # self.get_logger().info(f'YAW: {self.ebot_z}')

#     def controller_loop(self):
#         if self.is_controller_running_angular:

#             error_z = self.desired_z - self.ebot_z
#             print(self.ebot_z, error_z)
#             twist_msg = Twist()
#             if abs(error_z) > self.angular_tolerance:
#                 if self.ebot_z < 0:
#                     twist_msg.angular.z = -self.kp_angular*error_z
#                 else:
#                     twist_msg.angular.z = self.kp_angular*error_z
#                 twist_msg.linear.x = 0.0
#                 self.cmd_vel_publisher.publish(twist_msg)
#             else:
#                 twist_msg.linear.x = 0.0
#                 twist_msg.angular.z = 0.0
#                 self.cmd_vel_publisher.publish(twist_msg)
#                 self.get_logger().info("Oriented towards goal")
#                 self.is_controller_running_angular = False
#                 self.is_controller_running_linear = True
        
#         if self.is_controller_running_linear:
#             twist_msg = Twist()
#             error_x = self.desired_x - self.ebot_x
#             if error_x >= 0:
#                 linear_velocity = -self.kp_linear * error_x
#             else:
#                 linear_velocity = self.kp_linear * error_x
#             angular_velocity = 0.0

#             if abs(error_x) < self.linear_tolerance:
#                 twist_msg.linear.x = 0.0
#                 twist_msg.angular.z = 0.0
#                 self.get_logger().info(f'Reached Goal Successfully !!!')
#                 self.is_controller_running_linear = False
#             else:
#                 twist_msg.linear.x = linear_velocity
#                 twist_msg.angular.z = angular_velocity

#             self.cmd_vel_publisher.publish(twist_msg)

#     def start_docking_callback(self, request, response):
#         # self.get_logger().info(f'\n\nX: {str(self.ebot_x)}\tY: {str(self.ebot_y)}\tZ: {str(self.ebot_z)}')
#         self.is_controller_running_angular = True

#         response.success = True
#         response.message = "Controller started"
#         return response
    

# def main(args=None):
#     rclpy.init(args=args)

#     ebot_odom_dock_controller = EbotOdomDockController()

#     rclpy.spin(ebot_odom_dock_controller)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     ebot_odom_dock_controller.destroy_node()
#     # cv2.destroyAllWindows()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()