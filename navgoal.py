#!/usr/bin/env python3
#AYUSH KUMAR SHARMA
#ALL THE IMPORT STATEMENTS :-
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def pose_creator(navigator , position_x , position_y , rotation_z):
    goal_pose = PoseStamped()
    q_x , q_y , q_z ,q_w = tf_transformations.quaternion_from_euler(0.0,0.0,rotation_z)
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w

    return goal_pose




def main():
    rclpy.init()
    # -----------------------> THIS COMMENTED SECTION DOESNT USES THE UDF CREATED ABOVE <-----------------------------

    # initial_pose = PoseStamped() #will use to set the initial pose
    # goal_pose = PoseStamped()   #will use to set the fianl pose


    # q_x , q_y , q_z ,q_w = tf_transformations.quaternion_from_euler(0.0,0.0,0.0)

    # #----   this section will set the initial pose of the bot
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = nav.get_clock().now().to_msg()

    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.position.z = 0.0

    # initial_pose.pose.orientation.x = q_x
    # initial_pose.pose.orientation.y = q_y
    # initial_pose.pose.orientation.z = q_z
    # initial_pose.pose.orientation.w = q_w

    # nav.setInitialPose(initial_pose) #this will set the initial pose

    # #-----   this section will set the goal pose of the bot

    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp= nav.get_clock().now().to_msg()

    # goal_pose.pose.position.x = 2.5
    # goal_pose.pose.position.y = 0.0
    # goal_pose.pose.position.z = 0.0

    # goal_pose.pose.orientation.x = q_x
    # goal_pose.pose.orientation.y = q_y
    # goal_pose.pose.orientation.z = q_z
    # goal_pose.pose.orientation.w = q_w

    #---------------><-----------------------------#

    nav = BasicNavigator()

    initial_pose = pose_creator(nav , 0.0 , 0.0 , 0.0)
    nav.setInitialPose(initial_pose)

    nav.waitUntilNav2Active()

    goal_pose1 = pose_creator(nav , 2.5 , 0.0 , 0.0)
    goal_pose2 = pose_creator(nav , 1.5 , 5.0 , 0.0)
    goal_pose3 = pose_creator(nav , -3.5 , 9.0 , 0.0)


    waypoints = [goal_pose1 , goal_pose2 , goal_pose3]

    for i in range(3):
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
    

    print(nav.getResult())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
