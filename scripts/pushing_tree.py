#!/usr/bin/env python

import py_trees
import py_trees_ros
import py_trees_msgs.msg as py_trees_msgs
from geometry_msgs.msg import PoseStamped
import rospy
import pushing_behaviour_tree as pbt
import functools
import sys
import py_trees.console as console


def create_root():
    # behaviours
    root = py_trees.composites.Parallel(name="ROOT")
    # topic sequence
    topic_seq = py_trees.composites.Parallel(name="Topics2BB")
    # riporto sulla BB la posizione dell'oggetto
    obj_pose2BB = py_trees_ros.subscribers.ToBlackboard(
        name="obj2BB", topic_name="vrpn_client_node/Box/pose", topic_type=PoseStamped, blackboard_variables={'obj_pose': 'pose'})
    # riporto sulla BB la posizione del robot
    robot_pose2BB = py_trees_ros.subscribers.ToBlackboard(
        name="robot2BB", topic_name="vrpn_client_node/turtle5/pose", topic_type=PoseStamped, blackboard_variables={'robot_pose': 'pose'})
    # riporto sulla BB la posizione target dell'oggetto
    target_pose2BB = py_trees_ros.subscribers.ToBlackboard(
        name="target2BB", topic_name="target/pose", topic_type=PoseStamped, blackboard_variables={'target_pose': 'pose'})
    always_running = py_trees.behaviours.Running(name="AlwaysRunning")
    selector1 = py_trees.composites.Selector(name="Selector1")
    obj_at_target = pbt.CheckObjectInTarget(name="checkobjintarget")
    sequence1 = py_trees.composites.Sequence(name="Sequence1")
    selector2 = py_trees.composites.Selector(name="Selector2")
    check_push_traj = pbt.CheckPushingPaths(name="checkpushtraj")
    compute_traj = pbt.ComputeTrajectory(name="computepushtraj")
    sequence2 = py_trees.composites.Sequence(name="Sequence2")
    selector3 = py_trees.composites.Selector(name="Selector3")
    position_robot = pbt.PositionRobot(name="checkrobotposition")
    sequence3 = py_trees.composites.Sequence(name="Sequence3")
    selector4 = py_trees.composites.Selector(name="Selector4")
    robot_near_object = pbt.CheckRobotNearObject(name="checkrobotnearobject")
    detach_from_object = pbt.DetachfromObj(name="detach")
    move_to_approach = pbt.MoveToApproach(name="movetoapproach")
    approach_to_obj = pbt.Approach(name="approach")
    execute_pushing_traj = pbt.PushingTrajectory(name="executepushtraj")

    # struttura albero
    root.add_children([topic_seq, selector1])
    topic_seq.add_children([obj_pose2BB, robot_pose2BB, target_pose2BB])
    selector1.add_children([obj_at_target,sequence1])
    sequence1.add_children([selector2, sequence2])
    selector2.add_children([check_push_traj,compute_traj])
    sequence2.add_children([ selector3,execute_pushing_traj])
    selector3.add_children([position_robot, sequence3])
    sequence3.add_children([selector4,move_to_approach,approach_to_obj])
    selector4.add_children([ robot_near_object,detach_from_object])
    

    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    print("main")
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)

    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree,aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(200)


if __name__ == "__main__":
    main()
