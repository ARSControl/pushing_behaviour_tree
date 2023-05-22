import py_trees
import py_trees_ros
import py_trees_msgs.msg as py_trees_msgs
from geometry_msgs.msg import PoseStamped
import rospy
import components
import functools
import sys
import py_trees.console as console
def create_root():
	#behaviours
        root= py_trees.composites.Parallel("ROOT")
	# topic sequence
        topic_seq =py_trees.composites.Parallel("Topics2BB")
        #riporto sulla BB la posizione dell'oggetto
        obj_pose2BB =  py_trees_ros.subscribers.ToBlackboard(name="obj2BB",topic_name="obj/pose",topic_type=PoseStamped,blackboard_variables={'obj_pose':'pose'})
        #riporto sulla BB la posizione del robot
        robot_pose2BB = py_trees_ros.subscribers.ToBlackboard(name="robot2BB",topic_name="robot/pose",topic_type=PoseStamped,blackboard_variables={'robot_pose':'pose'})
        #riporto sulla BB la posizione target dell'oggetto
        target_pose2BB= py_trees_ros.subscribers.ToBlackboard(name="target2BB",topic_name="target/pose",topic_type=PoseStamped,blackboard_variables={'target_pose':'pose'})
        selector1=py_trees.composites.Selector("Selector") 
        obj_at_target = components.CheckObjectInTarget(name="cond1")
        sequence1=py_trees.composites.Sequence("Sequence")
        selector2=py_trees.composites.Selector("Selector")
        check_push_traj=components.CheckPushingPaths(name="cond4")
        compute_traj=components.ComputeTrajectory(name="action5")
        sequence2=py_trees.composites.Sequence("Sequence") 
        selector3=py_trees.composites.Selector("Selector")
        position_robot=components.PositionRobot("cond2")
        sequence3=py_trees.composites.Sequence("Sequence")
        selector4=py_trees.composites.Selector("Selector")
        robot_near_object = components.CheckRobotNearObject(name="cond3")
        detach_from_object= components.DetachfromObj(name="action1")
        move_to_approach=components.MoveToApproach(name="action2")
        approach_to_obj=components.Approach(name="action3") 
        sequence4=py_trees.composites.Sequence("Sequence")
        execute_pushing_traj= components.PushingTrajectory(name="action4")
        
        
        #struttura albero
        root.add_children([topic_seq,selector1])
        topic_seq.add_children([obj_pose2BB,robot_pose2BB,target_pose2BB])
        selector1.add_children([obj_at_target,sequence1])
        sequence1.addchildren([selector2,sequence2])
        selector2.addchildren([check_push_traj,compute_traj])
        sequence2.addchildren([selector3,sequence4])
        selector3.addchildren([position_robot,sequence3])
        sequence3.addchildren([selector4,move_to_approach,approach_to_obj])
        selector4.addchildren([robot_near_object,detach_from_object])
        sequence4.addchild([execute_pushing_traj])

        return root

def shutdown(behaviour_tree):
        behaviour_tree.interrupt()
        
def main():
        print("main")
        rospy.init_node("tree")
        root=create_root()
        behaviour_tree=py_trees_ros.trees.BehaviourTree(root)

        rospy.on_shutdown(functools.partial(shutdown,behaviour_tree))
        if not behaviour_tree.setup(timeout=15):
              console.logerror("failed to setup the tree,aborting.")
              sys.exit(1)
        behaviour_tree.tick_tock(200)
              


        
	
