#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Trigger
from linkattacher_msgs.srv import AttachLink
from ebot_docking.srv import DockSw
import math, statistics
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# from std_srvs.srv import Trigger


class TestDockLink(Node):

    def __init__(self):
        super().__init__('test_dock_link_attach')
        self.callback_group = ReentrantCallbackGroup()

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, qos_profile=1, callback_group=self.callback_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        # self.docking_cli = self.create_client(DockSw, 'start_docking')
        # while not self.docking_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Docking service not available, waiting again...')

        # self.dock_req = DockSw.Request()

        # self.link_attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        # while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Link attacher service not available, waiting again...')

        # self.link_attach_req = AttachLink.Request()

        # self.testing_docking_callback()

        # my_param= self.get_parameter("footprint").get_parameter_value()

        print(quaternion_from_euler(0,0,3.14))

        # self.local_cm_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        # self.local_cm_client.wait_for_service()

        # self.global_cm_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        # self.global_cm_client.wait_for_service()

        # self.clear_costmap_global_srv = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        # self.clear_costmap_local_srv = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # self.update_footprint(True)
        # self.clearGlobalCostmap()

        # self.clearLocalCostmap()
        self.ebot_pose=[0,0,0]

        self.move_a_bit(1.0)

    def odometry_callback(self, msg):
        self.odom_msg = msg

        self.ebot_pose[0] = msg.pose.pose.position.x
        self.ebot_pose[1] = msg.pose.pose.position.y

        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _,_, yaw = euler_from_quaternion(orientation_list)

        self.ebot_pose[2] = yaw

        print(self.ebot_pose)

    

    def move_a_bit(self, distance):

        goal_x = self.ebot_pose[0] + distance

        rate = self.create_rate(10)
            
            error_x = goal_x - self.ebot_pose[0]

            linear_velocity = 0.4 * error_x
            angular_velocity = 0.0

            twist_msg = Twist()

            if abs(error_x) >= 0.05:

                print("here")

                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity

                self.cmd_vel_publisher.publish(twist_msg)

            else:

                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

                self.cmd_vel_publisher.publish(twist_msg)

                self.get_logger().info(f'Reached Goal Successfully !!!')
                break
            
            # rate.sleep()


    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return
    
    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return
        

    def update_footprint(self, with_rack=False):

        self.get_logger().info(" Updating Footprint Request sending !!")

        request_local = SetParameters.Request()
        new_foot_print_local=Parameter()

        new_foot_print_local.name='footprint'
        new_foot_print_local.value.type = 4  ## For string type 4


        request_global = SetParameters.Request()
        new_foot_print_global=Parameter()

        new_foot_print_global.name='robot_radius'
        new_foot_print_global.value.type = 3 ## For double type 3
        
        # Update footprint with or without rack
        if with_rack:
            new_foot_print_local.value.string_value='[[0.41, 0.6], [0.41, -0.6], [-0.41, -0.6], [-0.41, 0.6] ]'
            new_foot_print_global.value.double_value = 0.7
        else:
            new_foot_print_local.value.string_value='[[0.21, 0.195], [0.21, -0.195], [-0.21, -0.195], [-0.21, 0.195] ]'
            new_foot_print_global.value.double_value = 0.3

        request_global.parameters = [new_foot_print_global]
        future_global=self.global_cm_client.call_async(request_global)
        rclpy.spin_until_future_complete(self, future_global)
        result_global = future_global.result().results[0].successful

        request_local.parameters = [new_foot_print_local]
        future_local=self.local_cm_client.call_async(request_local)
        rclpy.spin_until_future_complete(self, future_local)
        result_local = future_global.result().results[0].successful


        # print(result)
        if result_local:
            self.get_logger().info("Footprint Updated!!")
        else:
            self.get_logger().error("Footprint Updated Failed!!")


    def testing_docking_callback(self):

        self.get_logger().info(" DOCKING Request sending !!")

        self.dock_req.orientation=0.0
        self.dock_req.rack_no='rack1'

        future_dock=self.docking_cli.call_async(self.dock_req)


        rclpy.spin_until_future_complete(self, future_dock)

        print("DOCKING response: ", future_dock.result())


        self.get_logger().info(" DOCKING Request send !!")

        # response.success = True
        # response.message = "TESTING DONE !!!"
        # return response
    

def main(args=None):
    rclpy.init(args=args)

    test_dock_link_attach = TestDockLink()

    executor= MultiThreadedExecutor()
    executor.add_node(test_dock_link_attach)
    
    executor.spin()

    # rclpy.spin(test_dock_link_attach)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_dock_link_attach.destroy_node()
    # cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
