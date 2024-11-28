from franka_gripper.msg import *
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

def grasp_client():
    client = actionlib.SimpleActionClient('/panda1/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client.wait_for_server()

    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.022
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.1
    goal.force = 5

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult


if __name__ == '__main__':
    import time
    time.sleep(5)
    try:
        rospy.init_node('grasp_client_py')
        result = grasp_client()
        print("Success: ",result.success)
        print("Error message: ", result.error)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
