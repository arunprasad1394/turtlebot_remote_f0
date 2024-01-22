#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point

rospy.init_node('turtlebot_mover')


def increaseRobotSpeed():
    msgTwist = Twist()
    # The turtlebot Robot subscribe to cmd_vel topic so we send the cmd topic a linear speed(twist) so
    # the turtlebot will increase its speed 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msgTwist.linear.x = 12
    pub.publish(msgTwist)
    rospy.loginfo('speed increased...')

def CustomWayPoints():
    # Create the dictionary 
    locations = dict()
    # add our waypoint names and values. 
    locations['waypoint1'] = Pose(Point(-1.4811, -3.6870, -0.0062), Quaternion(0.000, 0.000, -0.396, 0.918)),
    locations['waypoint2'] = Pose(Point(4.0281, -2.4129, -0.0065),Quaternion(0.000, 0.000, -0.709, 0.704)),
    locations['waypoint3'] = Pose(Point(1.9178, 2.5220, -0.0060), Quaternion(0.000, 0.000, 0.622, 0.782)),
    return locations

# create a function that represnt posearray in Rviz so we could visualize the waypoints 
def wayPointsRviz(waypointsList):
    poseArrayPub= rospy.Publisher('/waypoints', PoseArray, queue_size=1)
    waypoints = PoseArray()
    waypoints.header.frame_id = 'map'
    waypointPoses = []
    for key, value in waypointsList.items():
        waypointPoses.append(waypointsList[key])

    waypoints.poses = waypointPoses
    poseArrayPub.publish(waypoints)

    return waypoints

def update_initial_pose(initial_pose):
        initial_pose = initial_pose

def sendGoals(waypoints):
    # With msg type PoseWithCovarianceStamped. 
    initial_pose = PoseWithCovarianceStamped()

    

    # set initial pose of the robot 
    initial_pose = PoseWithCovarianceStamped()
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, update_initial_pose)

    #This line of code is used when to get the initial position using RViz (The user needs to click on the map) 
    rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

    # subscribe to action server 
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # this command to wait for the server to start listening for goals.
    client.wait_for_server()

    # Show reuqired waypoints(red arrows of goals ) in Rviz
    wayPointsRviz(waypoints)

    # increase Robot speed while navigating 
    #while not rospy.is_shutdown():
    #increaseRobotSpeed()
    
    # Iterate over all the waypoits, follow the path 
    for key, value in waypoints.items():
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = waypoints[key].position.x
        goal.target_pose.pose.position.y = waypoints[key].position.y
        goal.target_pose.pose.position.z = waypoints[key].position.z
        # Goal Orientation
        goal.target_pose.pose.orientation.x = waypoints[key].orientation.x
        goal.target_pose.pose.orientation.y = waypoints[key].orientation.y
        goal.target_pose.pose.orientation.w = waypoints[key].orientation.z
        goal.target_pose.pose.orientation.z = waypoints[key].orientation.w

        client.send_goal(goal)
        wait = client.wait_for_result()
    rospy.loginfo('The waypoints path is complete')


ss = CustomWayPoints()
# gives back loactions
# print("print test")
# print(len(ss))
# print(wayPointsRviz(ss))

send = sendGoals(ss)

# print(wayPointsRviz(ss))