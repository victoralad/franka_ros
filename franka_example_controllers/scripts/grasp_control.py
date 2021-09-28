#!/usr/bin/env python
import argparse
import rospy
from std_msgs.msg import String
# from franka_gripper.msg import MoveActionGoal
from franka_gripper.msg import GraspActionGoal
# import ipdb


def sub_gripper(gp, pos, dt_sec):

    rospy.init_node('open_close_gripper', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # while not rospy.is_shutdown():
    control = GraspActionGoal()
    control.goal.width = pos
    control.goal.speed = 0.1
    control.goal.force = 25.0
    control.goal.epsilon.inner = 0.05
    control.goal.epsilon.outer = 0.05

    if (gp == 'l' or gp == 'L'):
        pubA = rospy.Publisher('/panda_left/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1, latch=True)
        pubA.publish(control)
    elif (gp == 'r' or gp == 'R'):
        pubB = rospy.Publisher('/panda_right/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1, latch=True)
        pubB.publish(control)
    else:
        pubA = rospy.Publisher('/panda_left/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1, latch=True)
        pubA.publish(control)
        pubB = rospy.Publisher('/panda_right/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1, latch=True)
        pubB.publish(control)

    print("pubishing ", pos)
    rospy.spin()

    # rate.sleep()

if __name__ == '__main__':
    # take 1 argyment 'data' which is an float in [0,1]
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('gp', help='gripper left or gripper right')
    parser.add_argument('pos', metavar='P', type=float, help='0.08 for open, 0.0 for close')
    parser.add_argument('--dt', type=float, default=1.0, help='number of seconds')
    
    args = parser.parse_args()
    pos = args.pos
    gp = args.gp
    dt_sec = args.dt

    # call callback(data)
    sub_gripper(gp, pos, dt_sec)