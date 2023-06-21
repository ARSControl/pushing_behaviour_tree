import py_trees
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt
import py_trees_msgs.msg as py_trees_msgs
from tf.transformations import euler_from_quaternion
from pushing_msgs.srv import RobotTrajectory, RobotTrajectoryRequest,  GetPushingPathsResponse, GetPushingPathsRequest,  GetPushingPaths
from nav_msgs.msg import OccupancyGrid, Path
import threading
import numpy as np


class CheckObjectInTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check1"):
        super(CheckObjectInTarget, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        print(self.blackboard)

    def setup(self, unused):
        self.feedback_message = "setup"
        self.blackboard.obj_pose = Pose()
        self.blackboard.target_pose = Pose()
        return True

    def update(self):
        obj = self.blackboard.obj_pose
        #print("object =")
        # print(obj.position)
        target = self.blackboard.target_pose
        #print("target =")
        # print(target.position)
        norm = sqrt((obj.position.x - target.position.x)**2 +
                    (obj.position.y - target.position.y)**2)
        if norm < 0.01:
            self.feedback_message = "object at target"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "object not at target"
        return py_trees.common.Status.FAILURE   

    def terminate(self, unused):
        pass


class PositionRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check2"):
        super(PositionRobot, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, unused):
        self.feedback_message = "setup"
        self.blackboard.sides = [0, 1, 2, 3]
        self.b = 0.12
        return True

    def update(self):
        side = self.blackboard.sides[0]
        robot = self.blackboard.robot_pose  # geometry_msgs/Pose
        thetar = 0.0#euler_from_quaternion(robot.orientation)[2]
        pr = np.array([[robot.position.x], [robot.position.y], [thetar]])
        obj = self.blackboard.obj_pose  # geometry_msgs/Pose
        thetao = 0.0#euler_from_quaternion(obj.orientation)[2]
        po = np.array([[obj.position.x], [obj.position.y], [thetao]])
        R = np.array([[math.cos(thetao), math.sin(thetao), 0],
                      [-math.sin(thetao), math.cos(thetao), 0],
                      [0, 0, 1]])
        pr_o = np.dot(R,(pr-po))

        xr_d = self.b*math.cos((2/5)*math.pi*side)
        yr_d = self.b*math.sin((2/5)*math.pi*side)
        thetar_d = math.pi + side*(2/5)*math.pi
        d = np.array([[xr_d], [yr_d], [thetar_d]]) - pr_o
        
        if (math.sqrt(d[0][0]*d[0][0] + d[1][0]*d[1][0]) < 0.01 )& (np.abs(d[2][0]) < 0.1):
            self.feedback_message = "object is in correct position"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "object is not in correct position"
            return py_trees.common.Status.FAILURE

    def terminate(self,new_status):
        pass


class CheckRobotNearObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check3"):
        super(CheckRobotNearObject, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        self.blackboard.obj_pose = Pose()
        self.blackboard.robot_pose = Pose()
        return True

    def update(self):
        obj = self.blackboard.obj_pose
        robot = self.blackboard.robot_pose
        dist = math.sqrt((obj.position.x - robot.position.x)**2 + (obj.position.y - robot.position.y)**2)
        if dist < 0.1:
            self.feedback_message = "robot near object"
            return py_trees.common.Status.FAILURE
        self.feedback_message = "robot far to object"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass


class DetachfromObj(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action1"):
        super(DetachfromObj, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        self.Ds = 0.250
        return True

    def initialise(self):
        robot = self.blackboard.robot_pose
        thetar = euler_from_quaternion(robot.orientation)[2]
        pr = np.array([robot.position.x, robot.position.y])
        obj = self.blackboard.obj_pose
        #thetao = euler_from_quaternion(obj.orientation)[2]
        po = np.array([obj.position.x, obj.position.y])
        x = pr-po
        V = (self.Ds-np.linalg.norm(x))*1.2
        self.blackboard.psafe = pr + V * \
            np.array([math.cos(thetar+math.pi), math.sin(thetar+math.pi)])

    def update(self):
        robot = self.blackboard.robot_pose
        pr = np.array([robot.position.x, robot.position.y])
        d = pr - self.blackboard.psafe
        if math.sqrt(d[0]**2 + d[1]**2) < 0.01:
            self.feedback_message = "robot is detached in first useful position"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass


class MoveToApproach(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action2"):
        super(MoveToApproach, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        self.blackboard.sides = [0, 1, 2, 3]
        return True

    def initalise(self):
        side = self.blackboard.sides[0]
        robot = self.blackboard.robot_pose
        #thetar = euler_from_quaternion(robot.orientation)[2]
        pr = np.array([robot.position.x, robot.position.y])
        obj = self.blackboard.obj_pose
        #thetao = euler_from_quaternion(obj.orientation)[2]
        po = np.array([obj.position.x, obj.position.y])
        #x= pr-po
        #V= (self.Ds-np.linalg.norm(x))*1.2
        # theta=np.array([math.cos(thetar+math.pi),math.sin(thetar+math.pi)])
        #self.blackboard.psafe= pr + V*theta
        d = np.linalg.norm(pr-po)
        xr_d = d*math.cos(math.pi*(2/5)*side)
        yr_d = d*math.sin(math.pi*(2/5)*side)
        thetar_d = math.pi + side*(2/5)*math.pi
        self.blackboard.pr_d = np.array([xr_d, yr_d, thetar_d])

    def update(self):
        robot = self.blackboard.robot_pose
        thetar = 0.0#euler_from_quaternion(robot.orientation)[2]
        pr = np.array([robot.position.x, robot.position.y, thetar])
        dd = self.blackboard.pr_d - pr
        if math.sqrt(dd[0]**2 + dd[1]**2) < 0.01 & math.abs(dd[2]) < 0.1:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass


class Approach(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action3"):
        super(Approach, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        self.blackboard.sides = [0, 1, 2, 3]
        self.b = 0.12
        return True

    def initialise(self):
        side = self.blackboard.sides[0]
        robot = self.blackboard.robot_pose
        thetar = euler_from_quaternion(robot.orientation)[2]
        pr = np.array([robot.position.x, robot.position.y, thetar])
        obj = self.blackboard.obj_pose
        thetao = euler_from_quaternion(obj.orientation)[2]
        po = np.array([obj.position.x, obj.position.y, thetao])
        R = np.array([[math.cos(thetao), math.sin(thetao), 0],
                      [-math.sin(thetao), math.cos(thetao), 0],
                      [0, 0, 1]])
        pr_o = R*(pr-po)
        xr_d = self.b*math.cos((2/5)*math.pi*side)
        yr_d = self.b*math.sin((2/5)*math.pi*side)
        thetar_d = math.pi + side*(2/5)*math.pi
        self.blackboard.d = np.array([xr_d, yr_d, thetar_d]) - pr_o

    def update(self):

        if math.sqrt(self.blackboard.d[0]**2 + self.blackboard.d[1]**2) < 0.01 & math.abs(self.blackboard.d[2]) < 0.1:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass


class PushingTrajectory(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action4"):
        super(PushingTrajectory, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
        # rospy.wait_for_service(
        #     'pushing_controller/SetTrajectory')
        # self.set_trajectory = rospy.ServiceProxy(
        #     'pushing_controller/SetTrajectory', RobotTrajectory)
        self.plan = Path
        return True

    def initialise(self):
        # rospy.wait_for_service('pushing_controller/SetTrajectory',RobotTrajectory)
        #set_trajectory = rospy.ServiceProxy('pushing_controller/SetTrajectory', RobotTrajectory)
        req = RobotTrajectoryRequest()
        req.length = 100
        req.constraints = 1
        req.trajectory = self.plan[0]
        # TODO set trajectory
        resp = self.set_trajectory(req)
        self.tend = rospy.Time.now() + rospy.Duration(self.req.length*0.1+2.5)
        # TODO set final target pose
        target = self.blackboard.target_pose
        thetat = euler_from_quaternion(target.orientation)[2]
        self.pt = np.array([target.position.x, target.position.y, thetat])
        return resp.ok

    def update(self):
        #target= self.blackboard.target_pose
        obj = self.blackboard.obj_pose
        thetao = euler_from_quaternion(obj.orientation)[2]
        po = np.array([obj.position.x, obj.position.y, thetao])
        #thetat = euler_from_quaternion(target.orientation)[2]
        #pt = np.array([target.position.x,target.position.y,thetat])
        d = self.pt - po

        if rospy.Time.now() < self.tend + rospy.Duration(10.0):
            return py_trees.common.Status.RUNNING
        else:
            if math.sqrt(d[0]**2 + d[1]**2) < 0.01 & math.abs(d[2] < 0.1):
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass


class ComputeTrajectory(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action5"):
        super(ComputeTrajectory, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self,unused):
        self.feedback_message = "setup"
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
        req = GetPushingPathsRequest()
        # Compose request
        req.start = self.obj  # posizione oggetto
        req.goal = self.target  # posizione target
        req.map = self.map_msg
        req.R_p = self.R_p  # raggio sterzata sinistra
        req.R_m = self.R_m  # raggio sterzata destra
        req.b_ = self.b  # distanza quando oggetto e robot sono vicini
        req.sides_nr = self.side_nr  # numero di lati (5 se è un pentagono)
        #self.t1 = threading.Thread(target=self.ask_planner, args=([req]))
        #self.t1.run()  # avvio il thread

    def update(self):
        self.feedback_message = "update"
        if True:#self.t1.is_alive():
            return py_trees.common.Status.SUCCESS

        else:
            if self.plan != []:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

    def terminate(self,unused):
        pass

    def ask_planner(self, req):
        with self.lock:
            self.plan = self.get_plan(req[0]) 

class CheckPushingPaths(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check4"):
        super(CheckPushingPaths, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, unused):
        self.feedback_message = "setup"
        return True

    def initialise(self):
        self.feedback_message = "initialize"
        self.resp = GetPushingPathsResponse()

    def update(self):
        self.feedback_message = "update"
        if self.resp.paths != []:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
