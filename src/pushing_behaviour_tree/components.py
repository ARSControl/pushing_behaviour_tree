import py_trees
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
from math import sqrt
import py_trees_msgs.msg as py_trees_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pushing_msgs.srv import RobotTrajectory, RobotTrajectoryRequest, GetPushingPathsRequest,  GetPushingPaths, SetTargetRequest, SetTarget, SetObstacles, SetObstaclesRequest
from nav_msgs.msg import OccupancyGrid, Path
import threading
import numpy as np


class CheckObjectInTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckObjinTarg"):
        super(CheckObjectInTarget, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        print(self.blackboard)

    def setup(self, unused):
        self.feedback_message = "setup"
        print("setup checkobjintarget")
        self.blackboard.obj_pose = Pose()
        self.blackboard.target_pose = Pose()
        return True

    def update(self):
        self.feedback_message = "update"
        print("update checkobjintarget")
        obj = self.blackboard.obj_pose
        target = self.blackboard.target_pose
        norm = sqrt((obj.position.x - target.position.x)**2 +
                    (obj.position.y - target.position.y)**2)
        if norm < 0.1:
            self.feedback_message = "object at target"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "object not at target"
        return py_trees.common.Status.FAILURE

    def terminate(self, unused):
        pass


class PositionRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckPositionRobot"):
        super(PositionRobot, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, unused):
        self.feedback_message = "setup"
        print("setup positionrobot")
        self.blackboard.sides = [0, 1, 2, 3]
        self.b_ = 0.12
        return True

    def update(self):
        self.feedback_message = "update"
        print("update positionrobot")
        side = self.blackboard.plan.sides[0]
        robot = self.blackboard.robot_pose
        q = [robot.orientation.x, robot.orientation.y,
                  robot.orientation.z,robot.orientation.w]
        thetar= euler_from_quaternion(q)[2]   
        pr = np.array([[robot.position.x], [robot.position.y], [thetar]])
        obj = self.blackboard.obj_pose 
        thetao = q = [obj.orientation.x, obj.orientation.y,
                      obj.orientation.z, obj.orientation.w]
        thetao = euler_from_quaternion(q)[2]
        xr_d = obj.position.x + self.b_*math.cos(thetao + (side * (2/4)*math.pi))
        yr_d = obj.position.y + self.b_*math.sin(thetao + (side * (2/4)*math.pi))
        thetar_d = thetao + (side*(2/4)*math.pi) + math.pi
        d = np.array([[xr_d], [yr_d], [thetar_d]]) - pr
        if (math.sqrt(d[0][0]*d[0][0] + d[1][0]*d[1][0]) < 0.01) & (np.abs(d[2][0]) < 0.1):
            self.feedback_message = "object is in correct position"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "object is not in correct position"
            return py_trees.common.Status.FAILURE

    def terminate(self,new_status):
        pass


class CheckRobotNearObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckRobotNear"):
        super(CheckRobotNearObject, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        print("setup checkrobotnearobj")
        self.blackboard.obj_pose = Pose()
        self.blackboard.robot_pose = Pose()
        return True

    def update(self):
        self.feedback_message = "update"
        print("update checkrobotnearobj")
        obj = self.blackboard.obj_pose
        robot = self.blackboard.robot_pose
        dist = math.sqrt((obj.position.x - robot.position.x) **2 
                         + (obj.position.y - robot.position.y) ** 2)
        if dist < 0.1:
            self.feedback_message = "robot near object"
            return py_trees.common.Status.FAILURE
        self.feedback_message = "robot far to object"
        return py_trees.common.Status.SUCCESS

    def terminate(self,new_status):
        pass


class DetachfromObj(py_trees.behaviour.Behaviour):
    def __init__(self, name="DetachfromObj"):
        super(DetachfromObj, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.client = rospy.ServiceProxy("/pushing_controller/SetTarget",   SetTarget)
        self.pub = rospy.Publisher("pushing_tree/robot_target", PoseStamped, queue_size=1)
        
    def setup(self,unused):
        self.feedback_message = "setup"
        print("setup detachfromobj")
        self.Ds = 0.250
        return True

    def initialise(self):
        self.feedback_message = "initialise"
        print("initialise detach")
        robot = self.blackboard.robot_pose
        q= [robot.orientation.x,robot.orientation.y,
            robot.orientation.z,robot.orientation.w]
        thetar = euler_from_quaternion(q)[2]
        pr = np.array([robot.position.x, robot.position.y])
        obj = self.blackboard.obj_pose
        po = np.array([obj.position.x, obj.position.y])
        x = pr-po
        V = (self.Ds-np.linalg.norm(x))*1.2
        self.blackboard.psafe = pr + V * \
            np.array([math.cos(thetar+math.pi), math.sin(thetar+math.pi)])
        req = SetTargetRequest()
        req.enable_constraints = 2
        req.target.x = self.blackboard.psafe[0]
        req.target.y = self.blackboard.psafe[1]
        req.target.theta = thetar
        p_msg = PoseStamped()
        p_msg.pose.position.x = float(req.target.x)
        p_msg.pose.position.y = float(req.target.y)
        q = quaternion_from_euler(0, 0, thetar)
        p_msg.pose.orientation.x = q[0]
        p_msg.pose.orientation.y = q[1]
        p_msg.pose.orientation.z = q[2]
        p_msg.pose.orientation.w = q[3]
        self.pub.publish(p_msg)
        self.client(req)
        self.tend = rospy.Time.now()

    def update(self):
        self.feedback_message = "update"
        print("update detach")
        robot = self.blackboard.robot_pose
        pr = np.array([robot.position.x, robot.position.y])
        d = pr - self.blackboard.psafe
        if rospy.Time.now() < self.tend + rospy.Duration(30):
            if math.sqrt(d[0]**2 + d[1]**2) < 0.01:
                self.feedback_message = "robot is detached in first useful position"
                return py_trees.common.Status.SUCCESS
            else:
                 return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE

    def terminate(self,new_status):
        pass

class MoveToApproach(py_trees.behaviour.Behaviour):
    def __init__(self, name="MovetoAppr"):
        super(MoveToApproach, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.client = rospy.ServiceProxy("/pushing_controller/SetTarget",   SetTarget)
        self.set_obstacles = rospy.ServiceProxy("pushing_controller/SetObstacles",   SetObstacles)
        self.pub = rospy.Publisher("pushing_tree/robot_target", PoseStamped, queue_size=1)
        self.pr_d = []

    def setup(self,unused):
        self.feedback_message = "setup"
        print("setup movetoappr")
        self.blackboard.sides = [0, 1, 2, 3]
        self.obst = Pose()
        self.obstacles = self.blackboard.get("obstacles")
        return True
    
    def initialise(self):
        print("initialize movetoappr")
        self.feedback_message = "initialize"
        side = self.blackboard.plan.sides[0]
        obj = self.blackboard.obj_pose
        self.obst.position.x = obj.position.x
        self.obst.position.y = obj.position.y
        self.obstacles.append(self.obst)
        req = SetObstaclesRequest()
        req.obstacles = self.obstacles
        self.set_obstacles(req)
        q = [obj.orientation.x, obj.orientation.y,
             obj.orientation.z, obj.orientation.w]
        thetao = euler_from_quaternion(q)[2]
        xr_d = obj.position.x + 0.25*math.cos(thetao + (side*(2/4)*math.pi))
        yr_d= obj.position.y + 0.25*math.sin(thetao + (side *(2/4)* math.pi))
        thetar_d =math.pi + side*(2/4)*math.pi
        thetar_d = np.arctan2(np.sin(thetar_d),np.cos(thetar_d))
        self.pr_d = np.array([xr_d, yr_d, thetar_d])
        req = SetTargetRequest()
        req.enable_constraints = 2
        req.target.x = xr_d
        req.target.y = yr_d
        req.target.theta = thetar_d
        p_msg = PoseStamped()
        p_msg.pose.position.x = req.target.x
        p_msg.pose.position.y = req.target.y
        q = quaternion_from_euler(0,0,thetar_d)
        p_msg.pose.orientation.x = q[0]
        p_msg.pose.orientation.y = q[1]
        p_msg.pose.orientation.z = q[2]
        p_msg.pose.orientation.w = q[3]
        self.pub.publish(p_msg)
        self.client(req)
        self.tend = rospy.Time.now()

    def update(self):
        self.feedback_message = "update"
        print("update movetoapproach")
        robot = self.blackboard.robot_pose
        q = [robot.orientation.x, robot.orientation.y,
             robot.orientation.z,robot.orientation.w]
        thetar = euler_from_quaternion(q)[2]
        pr = np.array([robot.position.x, robot.position.y, thetar])
        print(pr)
        print(self.pr_d)
        dd = pr - self.pr_d
        if rospy.Time.now() < self.tend + rospy.Duration(30):
            if math.sqrt(dd[0]**2 + dd[1]**2) < 0.01 and np.abs(dd[2])<0.1:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE
            

    def terminate(self,new_status):
        pass

class Approach(py_trees.behaviour.Behaviour):
    def __init__(self,name= "Approach"):
        super(Approach,self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.client = rospy.ServiceProxy("/pushing_controller/SetTarget",   SetTarget)
        self.pub = rospy.Publisher("pushing_tree/robot_target", PoseStamped, queue_size=1)
        self.prd2 = []
    def setup(self,unused):
        self.feedback_message = "setup"
        print("setup approach")
        self.b_ = 0.12
        self.blackboard.sides = [0, 1, 2, 3]
        return True
    def initialise(self):
        self.feedback_message = "initalise"
        print("initialise approach")
        side = self.blackboard.plan.sides[0]
        obj = self.blackboard.obj_pose
        print(obj)
        q = [obj.orientation.x,obj.orientation.y,
             obj.orientation.z, obj.orientation.w]
        thetao = euler_from_quaternion(q)[2]
        xr_2 = obj.position.x + self.b_*math.cos(thetao + (side * (2/4)*math.pi))
        yr_2 =  obj.position.y +self.b_*math.sin(thetao + (side * (2/4)* math.pi))
        thetar_2 = thetao + (side*(2/4)*math.pi) + math.pi
        thetar_2 = np.arctan2(np.sin(thetar_2),np.cos(thetar_2))
        self.prd2 = np.array([xr_2,yr_2,thetar_2])
        req = SetTargetRequest()
        req.enable_constraints = 2
        req.target.x = xr_2
        req.target.y = yr_2
        req.target.theta = thetar_2
        p_msg = PoseStamped()
        p_msg.pose.position.x = float(req.target.x)
        p_msg.pose.position.y = float(req.target.y)
        q = quaternion_from_euler(0,0,thetar_2)
        p_msg.pose.orientation.x = q[0]
        p_msg.pose.orientation.y = q[1]
        p_msg.pose.orientation.z = q[2]
        p_msg.pose.orientation.w = q[3]
        self.pub.publish(p_msg)
        self.client(req)
        self.tend = rospy.Time.now()


    def update(self):
        self.feedback_message = "update"
        print("update approach")
        robot = self.blackboard.robot_pose
        q = [robot.orientation.x, robot.orientation.y,
             robot.orientation.z,robot.orientation.w]
        thetar = euler_from_quaternion(q)[2]
        pr = np.array([robot.position.x,robot.position.y, thetar])
        obj = self.blackboard.obj_pose
        q = [obj.orientation.x, obj.orientation.y,
             obj.orientation.z, obj.orientation.w]
        print(pr)
        print(self.prd2)
        d = pr- self.prd2
        if rospy.Time.now() < self.tend + rospy.Duration(30):
            if math.sqrt(d[0]**2 + d[1]**2)< 0.01 and np.abs(d[2])<0.1:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE
    
    def terminate(self,new_status):
        pass




class PushingTrajectory(py_trees.behaviour.Behaviour):
    def __init__(self, name="PushingTraj"):
        super(PushingTrajectory, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
    def setup(self,unused):
        self.feedback_message = "setup"
        print("setup pushtraj")
        self.b_ = 0.12
        self.pub = rospy.Publisher("pushing_tree/object_target", Path, queue_size=1 )
        return True
 
    def path_to_trajectory_msg(self,path):
        traj = Float64MultiArray()
        q = path.poses[0].pose.orientation
        old_theta = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        for pose in path.poses:
            q = pose.pose.orientation
            theta = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
            traj.data.append(pose.pose.position.x + self.b_*math.cos(theta))
            traj.data.append(pose.pose.position.y + self.b_*math.sin(theta))
            traj.data.append(theta)
            traj.data.extend([0.05,((theta-old_theta)/0.1)])
            old_theta = theta
        return traj

    def initialise(self):
        self.feedback_message = "initialise"
        print("initialise pushtraj")
        rospy.wait_for_service('pushing_controller/SetTrajectory',10)
        self.set_trajectory = rospy.ServiceProxy('pushing_controller/SetTrajectory', RobotTrajectory)
        self.plan = self.blackboard.get("plan")
        print("initilize pushing")
        self.path_target_ = self.plan.paths.pop(0)
        self.pub.publish(self.path_target_)
        self.plan.sides.pop(0)
        req = RobotTrajectoryRequest()
        req.length = len(self.path_target_.poses)
        req.constraints = 1
        req.trajectory = self.path_to_trajectory_msg(self.path_target_)
        resp = self.set_trajectory(req)
        self.tend = rospy.Time.now() + rospy.Duration(req.length*0.1+2.5)
        target = self.path_target_.poses[-1].pose
        q = [target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w]
        thetat = euler_from_quaternion(q)[2]
        self.blackboard.pt = np.array([target.position.x, target.position.y, thetat])

    def update(self):
        self.feedback_message = "update"
        print("update pushtraj")
        obj = self.blackboard.obj_pose
        q = [obj.orientation.x, obj.orientation.y,
             obj.orientation.z, obj.orientation.w]
        thetao = euler_from_quaternion(q)[2]
        po = np.array([obj.position.x, obj.position.y, thetao])
        d = po - self.blackboard.pt

        if rospy.Time.now() < self.tend + rospy.Duration(10.0):
            return py_trees.common.Status.RUNNING
        else:
            if math.sqrt(d[0]**2 + d[1]**2) < 0.01 and np.abs(d[2] < 0.1):
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
            
    def terminate(self,unused):
        pass


class ComputeTrajectory(py_trees.behaviour.Behaviour):
    def __init__(self, name="ComputeTraj"):
        super(ComputeTrajectory, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        print("setup computetraj")
        self.get_plan = rospy.ServiceProxy(
            '/pushing_planner/GetPushingPlan', GetPushingPaths)
        self.plan = []
        self.target = self.blackboard.target_pose
        self.obj = self.blackboard.obj_pose
        self.b = 0.12
        self.map_msg = OccupancyGrid()
        self.R_p = 0.57
        self.R_m = -0.57
        self.side_nr = 5
        self.lock = threading.Lock()
        return True

    def initialise(self):
        self.feedback_message = "initialize"
        print("initialise computetraj")
        req = GetPushingPathsRequest()
        # Compose request
        req.start.x = self.obj.position.x  # posizione oggetto
        req.start.y = self.obj.position.y
        q = [self.obj.orientation.x,self.obj.orientation.y,
             self.obj.orientation.z,self.obj.orientation.w]
        req.start.theta = euler_from_quaternion(q)[2] #posizione oggetto
        req.goal.x = self.target.position.x 
        req.goal.y = self.target.position.y
        q = [self.target.orientation.x, self.target.orientation.y,
             self.target.orientation.z, self.target.orientation.w]
        req.goal.theta = euler_from_quaternion(q)[2]  # posizione target
        req.map = self.map_msg
        req.R_p = self.R_p  # raggio sterzata sinistra
        req.R_m = self.R_m  # raggio sterzata destra
        req.b_ = self.b  # distanza quando oggetto e robot sono vicini
        req.sides_nr = self.side_nr  # numero di lati (5 se è un pentagono)
        self.t1 = threading.Thread(target=self.ask_planner, args=([req]))
        self.t1.run()  # avvio il thread

    def update(self):
        self.feedback_message = "update"
        print("update computetraj")
        if self.t1.is_alive():
            return py_trees.common.Status.RUNNING
        if self.plan != []:
            self.feedback_message = "traj is computed"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
    def terminate(self,unused):
        pass

    def ask_planner(self, req):
        with self.lock:
            self.plan = self.get_plan(req)
            self.plan.sides = list(self.plan.sides)
            self.blackboard.set("plan",self.plan)


class CheckPushingPaths(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckPushTraj"):
        super(CheckPushingPaths, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, unused):
        self.feedback_message = "setup"
        print("setup checkpushtraj")
        return True

    def initialise(self):
        self.feedback_message = "initialize"
        print("initialise checkpushtraj")
        self.resp = self.blackboard.get("plan")
    

    def update(self):
        self.feedback_message = "update"
        print("update checkpushtraj")
        if self.resp :
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


