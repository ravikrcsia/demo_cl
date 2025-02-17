#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from ebot_docking.srv import DockSw
from linkattacher_msgs.srv import AttachLink, DetachLink
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


class EbotNavigate(Node):
    def __init__(self):
        super().__init__('ebot_nav_node')
        self.callback_group = ReentrantCallbackGroup()


        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize action client
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info('Waiting for Navigate to pose action server to be available...')
        self.action_client.wait_for_server()

        self.get_logger().info('Navigate to pose action server available')

        self.docking_cli = self.create_client(DockSw, '/start_docking')
        while not self.docking_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Docking service not available, waiting again...')

        self.get_logger().info('Docking service available')

        self.link_attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')

        self.link_attach_req = AttachLink.Request()

        self.link_detach_cli = self.create_client(DetachLink, '/DETACHLINK')
        while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link detacher service not available, waiting again...')

        self.link_detach_req = DetachLink.Request()

        self.get_logger().info('Link Detacher service available')

        self.local_cm_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.local_cm_client.wait_for_service()

        self.global_cm_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        self.global_cm_client.wait_for_service()

        self.max_vel_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.max_vel_client.wait_for_service()


        self.dock_req = DockSw.Request()

        self.home_pose=(0.0, 0.0, 0.0)
        self.ebot_pose=[0.0,0.0,0.0]
        
        self.move_ahead_flag=False
        self.move_distance_ahead=0.0 ##Store goal pose

        # self.goals_src={'rack2': (-4.38, 2.54, -1.57), 'rack7': (1.85, -7.0, 1.57)}
        # self.goals_des={'goal1': (1.0, -2.3, 3.14), 'goal2': (1.7, -4.0, -1.57)}

        self.goals_src={'rack7': (1.85, -7.0, 1.57)}
        self.goals_des={'goal1': (1.0, -2.3, 3.14)}

        self.current_goal=0

        # self.move_ahead_a_bit(1.0)

        self.nav_and_dock()

        self.move_home()

    def odometry_callback(self, msg):
        self.odom_msg = msg

        self.ebot_pose[0] = msg.pose.pose.position.x
        self.ebot_pose[1] = msg.pose.pose.position.y

        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _,_, yaw = euler_from_quaternion(orientation_list)

        self.ebot_pose[2] = yaw

        self.move_ahead()

        # print(self.ebot_pose)

    def move_ahead(self):

        if self.move_ahead_flag:
            error_x = self.move_distance_ahead - self.ebot_pose[0]

            linear_velocity = 0.5 * error_x
            angular_velocity = 0.0

            twist_msg = Twist()

            if abs(error_x) >= 0.05:

                print("here", error_x)

                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity

                self.cmd_vel_publisher.publish(twist_msg)

            else:

                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

                self.cmd_vel_publisher.publish(twist_msg)

                self.get_logger().info(f'Reached Goal Successfully !!!')
                self.move_ahead_flag=False

    def move_ahead_a_bit(self, distance):

        self.move_ahead_flag=True
        self.move_distance_ahead=self.ebot_pose[0] + distance

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

        request_max_vel = SetParameters.Request()
        max_vel=Parameter()

        max_vel.name='FollowPath.max_vel_x'
        max_vel.value.type = 3 ## For double type 3

        request_x_tol= SetParameters.Request()
        request_yaw_tol= SetParameters.Request()
        x_tol=Parameter()
        yaw_tol=Parameter()

        x_tol.name='general_goal_checker.xy_goal_tolerance'
        x_tol.value.type = 3 ## For double type 3

        yaw_tol.name='general_goal_checker.yaw_goal_tolerance'
        yaw_tol.value.type = 3 ## For double type 3
        
        # Update footprint with or without rack
        if with_rack:
            new_foot_print_local.value.string_value='[[0.41, 0.6], [0.41, -0.6], [-0.41, -0.6], [-0.41, 0.6] ]'
            new_foot_print_global.value.double_value = 0.7
            max_vel.value.double_value = 0.26

            x_tol.value.double_value = 0.30
            yaw_tol.value.double_value = 0.15

        else:
            new_foot_print_local.value.string_value='[[0.21, 0.195], [0.21, -0.195], [-0.21, -0.195], [-0.21, 0.195] ]'
            new_foot_print_global.value.double_value = 0.3
            max_vel.value.double_value = 0.35

            x_tol.value.double_value = 0.20
            yaw_tol.value.double_value = 0.15

        # request_global.parameters = [new_foot_print_global]
        # future_global=self.global_cm_client.call_async(request_global)
        # rclpy.spin_until_future_complete(self, future_global)
        # result_global = future_global.result().results[0].successful

        # request_local.parameters = [new_foot_print_local]
        # future_local=self.local_cm_client.call_async(request_local)
        # rclpy.spin_until_future_complete(self, future_local)
        # result_local = future_global.result().results[0].successful

        request_max_vel.parameters = [max_vel]
        future_max_vel=self.max_vel_client.call_async(request_max_vel)
        rclpy.spin_until_future_complete(self, future_max_vel)
        result_max_vel = future_max_vel.result().results[0].successful

        request_x_tol.parameters = [x_tol]
        future_x_tol=self.max_vel_client.call_async(request_x_tol)
        rclpy.spin_until_future_complete(self, future_x_tol)
        result_x_tol = future_x_tol.result().results[0].successful

        request_yaw_tol.parameters = [yaw_tol]
        future_yaw_vel=self.max_vel_client.call_async(request_yaw_tol)
        rclpy.spin_until_future_complete(self, future_yaw_vel)
        result_yaw_tol = future_yaw_vel.result().results[0].successful


        # print(result)
        if result_yaw_tol:
            self.get_logger().info("Footprint and Max_Vel Updated!!")
        else:
            self.get_logger().error("Footprint and Max_Vel Update Failed!!")

    def docking(self, rack, ori):

        self.get_logger().info(" DOCKING Request sending !!")

        self.dock_req.orientation=ori
        self.dock_req.rack_no=rack

        future_dock=self.docking_cli.call_async(self.dock_req)


        rclpy.spin_until_future_complete(self, future_dock)

        print("DOCKING response: ", future_dock.result())

        self.get_logger().info(" DOCKING Request send !!")

    def attach_with_ebot(self, rack, link='link'):

        self.get_logger().info(" Link Attach Request sending !!")

        id=rack[-1]

        ##Attach package with rack
        self.link_attach_req.model1_name = 'package_box'+'_'+str(id)
        self.link_attach_req.link1_name = 'link'
        self.link_attach_req.model2_name = rack
        self.link_attach_req.link2_name = link

        future_link=self.link_attach_cli.call_async(self.link_attach_req)
        rclpy.spin_until_future_complete(self, future_link)

        ##Attach rack with ebot

        self.link_attach_req.model1_name = 'ebot'
        self.link_attach_req.link1_name = 'base_link'
        self.link_attach_req.model2_name = rack
        self.link_attach_req.link2_name = link

        future_link=self.link_attach_cli.call_async(self.link_attach_req)
        rclpy.spin_until_future_complete(self, future_link)

        print("Linking response: ", future_link.result())

        self.get_logger().info(" Link Attach Request send !!")

    def detach_with_ebot(self, rack, link='link'):

        self.get_logger().info(" Link Detach Request sending !!")

        id=rack[-1]

        ##Detach package with rack
        self.link_detach_req.model1_name = 'package_box'+'_'+str(id)
        self.link_detach_req.link1_name = 'link'
        self.link_detach_req.model2_name = rack
        self.link_detach_req.link2_name = link

        future_link=self.link_detach_cli.call_async(self.link_detach_req)
        rclpy.spin_until_future_complete(self, future_link)

        ##Detach rack with ebot
        self.link_detach_req.model1_name = 'ebot'
        self.link_detach_req.link1_name = 'base_link'
        self.link_detach_req.model2_name = rack
        self.link_detach_req.link2_name = link

        future_link=self.link_detach_cli.call_async(self.link_detach_req)
        rclpy.spin_until_future_complete(self, future_link)

        print("De-linking response: ", future_link.result())

        self.get_logger().info(" Link Detach Request send !!")

    def detach_with_rack_base(self, rack, link='link'):

        self.get_logger().info(" Link Detach Request sending !!")

        id=rack[-1]

        ##Detach rack with ebot
        self.link_detach_req.model1_name = 'rack'+str(id)
        self.link_detach_req.link1_name = 'link'
        self.link_detach_req.model2_name = 'rack_base'+str(id)
        self.link_detach_req.link2_name = link

        future_link=self.link_detach_cli.call_async(self.link_detach_req)
        rclpy.spin_until_future_complete(self, future_link)

        print("De-linking response: ", future_link.result())

        self.get_logger().info(" Link Detach Request send !!")

    def move_home(self):

        self.get_logger().info('Moving to HOME !')

        print(self.send_navigation_goal(self.home_pose[0], self.home_pose[1], self.home_pose[2]))

    

    def nav_and_dock(self):

        rate = self.create_rate(2, self.get_clock())
        
        while self.current_goal<len(self.goals_src):
            c=self.current_goal
            goal_start=self.goals_src[list(self.goals_src)[c]]
            goal_end=self.goals_des[list(self.goals_des)[c]]

            ## Move to rack_goal
            print(self.send_navigation_goal(goal_start[0], goal_start[1], goal_start[2]))

            ## Start docking
            rack=list(self.goals_src)[c]
            ori=self.goals_src[list(self.goals_src)[c]][2]
            self.docking(rack, ori)

            self.attach_with_ebot(list(self.goals_src)[c])
            self.detach_with_rack_base(list(self.goals_src)[c])

            self.update_footprint(True)

            ## Move_ahead a bit

            # self.move_ahead_a_bit(0.5)

            # while self.move_ahead_flag:
            #     self.get_logger().info(" Moving ahead a bit")
            #     rate.sleep()

            ## Move to arm
            print(self.send_navigation_goal(goal_end[0], goal_end[1], goal_end[2]))

            ## start Undocking/detach rack
            self.detach_with_ebot(list(self.goals_src)[c])
            self.update_footprint(False)

            self.current_goal+=1

        rate.sleep()

    def goal_msg_wrap(self, x,y,yaw):
        goal_msg = NavigateToPose.Goal()
        # Set your desired pose here
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0,0, yaw)

        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        return goal_msg


    def send_navigation_goal(self, x, y, yaw):

        goal_msg = self.goal_msg_wrap(x,y, yaw)
        
        future = self.action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle.accepted:
            self.get_logger().info('Goal accepted :)')

            get_result_future = goal_handle.get_result_async()

            rclpy.spin_until_future_complete(self, get_result_future)

            status = future.result().status
            future.result()

            if status == 4:
                self.get_logger().info('Goal reached!')
                return True
            else:
                self.get_logger().error(f'Failed to reach goal with status code: {status}')
                return False

        else:
            self.get_logger().error('Goal rejected :(')
            return False

        # future.add_done_callback(self.goal_response)

    # def goal_response(self, future):
    #     goal_handle = future.result()

    #     if not goal_handle.accepted:
            
    #         return

        
    #     get_result_future = goal_handle.get_result_async()
    #     get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     status = future.result().status
    #     if status == 4:
    #         self.get_logger().info('Goal reached!')
    #         # start EM lock first
    #         # self.get_logger().info('Heading towards docking')
    #         # self.is_controller_running_angular = True   # start the controller loop for docking

    #     else:
    #         self.get_logger().error(f'Failed to reach goal with status code: {status}')

def main(args=None):
    rclpy.init(args=args)
    ebot_nav_node = EbotNavigate()
    rclpy.spin(ebot_nav_node)
    ebot_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()