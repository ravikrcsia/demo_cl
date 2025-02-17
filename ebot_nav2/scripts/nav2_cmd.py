#! /usr/bin/env python3


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import euler_from_quaternion, quaternion_from_euler

"""
Basic navigation demo to go to pose.
"""

def nav_goals(navigator,x,y,yaw):
    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    
    qx, qy, qz, qw = quaternion_from_euler(0,0, yaw)

    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            # print('Estimated time of arrival: ' + '{0:.0f}'.format(
            #       Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            #       + ' seconds.')
            
            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        # print('Goal succeeded!')
        return True
    elif result == TaskResult.CANCELED:
        # print('Goal was canceled!')
        return False
    elif result == TaskResult.FAILED:
        # print('Goal failed!')
        return False
    else:
        print('Goal has an invalid return status!')


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    if nav_goals(navigator, 1.8, 1.5, 1.57):
        print('Goal 1 succeeded!')

    if nav_goals(navigator, 2.0, -7.0, -1.57):
        print('Goal 2 succeeded!')

    if nav_goals(navigator, -3.0, 2.5, 1.57):
        print('Goal 3 succeeded!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
