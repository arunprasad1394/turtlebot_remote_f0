#!/usr/bin/env python


import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion


class TurtlebotMover:
   def __init__(self):
       rospy.init_node('turtlebot_mover', anonymous=True)


       # Define three positions to move the robot to
       positions = [
           [(-1.4811, -3.6870, -0.0062), (0.0, 0.0, -0.38, 0.923)],  # Position 1
           [(4.0281, -2.4129, -0.0065), (0.0, 0.0, -0.8184, 0.574)],  # Position 2
           [(1.9178, 2.5220, -0.0060), (0.0, 0.0, 0.79, 0.603)]  # Position 3
       ]


       # Create an action client for move_base
       self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
       self.move_base_client.wait_for_server()


       # Move the robot to the predefined positions
       for position in positions:
           self.move_to_position(position)


   def move_to_position(self, position):
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id = 'map'
       goal.target_pose.header.stamp = rospy.Time.now()


       # Set position
       goal.target_pose.pose.position = Point(*position[0])


       # Set orientation
       goal.target_pose.pose.orientation = Quaternion(*position[1])


       # Send goal to move_base
       self.move_base_client.send_goal(goal)


       # Wait for the robot to reach the goal
       self.move_base_client.wait_for_result()


       # Check if the robot reached the goal
       if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
           rospy.loginfo("Robot reached the goal!")
       else:
           rospy.loginfo("Failed to reach the goal.")


if __name__ == '__main__':
   try:
       turtlebot_mover = TurtlebotMover()
   except rospy.ROSInterruptException:
       pass




