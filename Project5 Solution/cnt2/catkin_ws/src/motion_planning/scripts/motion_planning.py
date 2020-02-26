#!/usr/bin/env python

from copy import deepcopy
import math
import numpy
import random
from threading import Thread, Lock
import sys

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

def convert_from_message(msg):
    R = tf.transformations.quaternion_matrix((msg.orientation.x,
                                              msg.orientation.y,
                                              msg.orientation.z,
                                              msg.orientation.w))
    T = tf.transformations.translation_matrix((msg.position.x, 
                                               msg.position.y, 
                                               msg.position.z))
    return numpy.dot(T,R)

def convert_from_trans_message(msg):
    R = tf.transformations.quaternion_matrix((msg.rotation.x,
                                              msg.rotation.y,
                                              msg.rotation.z,
                                              msg.rotation.w))
    T = tf.transformations.translation_matrix((msg.translation.x, 
                                               msg.translation.y, 
                                               msg.translation.z))
    return numpy.dot(T,R)
   


class MoveArm(object):

    def __init__(self):
        print "Motion Planning Initializing..."
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # Some info and conventions about the robot that we hard-code in here
        # min and max joint values are not read in Python urdf, so we must hard-code them here
        self.num_joints = 7
        self.q_min = []
        self.q_max = []
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        self.q_min.append(-3.1459);self.q_max.append(3.1459)
        # How finely to sample each joint
        self.q_sample = [0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1]
        self.joint_names = ["lwr_arm_0_joint",
                            "lwr_arm_1_joint",
                            "lwr_arm_2_joint",
                            "lwr_arm_3_joint",
                            "lwr_arm_4_joint",
                            "lwr_arm_5_joint",
                            "lwr_arm_6_joint"]

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, 
                         self.joint_states_callback)

        # Subscribe to command for motion planning goal
        rospy.Subscriber("/motion_planning_goal", geometry_msgs.msg.Transform,
                         self.move_arm_cb)

        # Publish trajectory command
        self.pub_trajectory = rospy.Publisher("/joint_trajectory", trajectory_msgs.msg.JointTrajectory, 
                                              queue_size=1)        

        # Initialize variables
        self.joint_state = sensor_msgs.msg.JointState()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "lwr_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name) 
        print "MoveIt! interface ready"

        # Options
        self.subsample_trajectory = True
        print "Initialization done."

    def get_joint_val(self, joint_state, name):
        if name not in joint_state.name:
            print "ERROR: joint name not found"
            return 0
        i = joint_state.name.index(name)
        return joint_state.position[i]

    def set_joint_val(self, joint_state, q, name):
        if name not in joint_state.name:
            print "ERROR: joint name not found"
        i = joint_state.name.index(name)
        joint_state.position[i] = q

    """ Given a complete joint_state data structure, this function finds the values for 
    our arm's set of joints in a particular order and returns a list q[] containing just 
    those values.
    """
    def q_from_joint_state(self, joint_state):
        q = []
        for i in range(0,self.num_joints):
            q.append(self.get_joint_val(joint_state, self.joint_names[i]))
        return q

    """ Given a list q[] of joint values and an already populated joint_state, this 
    function assumes that the passed in values are for a our arm's set of joints in 
    a particular order and edits the joint_state data structure to set the values 
    to the ones passed in.
    """
    def joint_state_from_q(self, joint_state, q):
        for i in range(0,self.num_joints):
            self.set_joint_val(joint_state, q[i], self.joint_names[i])

    """ This function will perform IK for a given transform T of the end-effector. It 
    returns a list q[] of 7 values, which are the result positions for the 7 joints of 
    the left arm, ordered from proximal to distal. If no IK solution is found, it 
    returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "world_link"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = self.q_from_joint_state(res.solution.joint_state)
        return q

    """ This function checks if a set of joint angles q[] creates a valid state, or 
    one that is free of collisions. The values in q[] are assumed to be values for 
    the joints of the left arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        current_joint_state = deepcopy(self.joint_state)
        current_joint_state.position = list(current_joint_state.position)
        self.joint_state_from_q(current_joint_state, q)
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = current_joint_state
        res = self.state_valid_service(req)
        return res.valid
        
     
        
    def motion_plan(self, q_start, q_goal, q_min, q_max):
        
        # Replace this with your code
        
        # Initiate a configuration space tree dictionary
        q_start=numpy.array(q_start) # Numpy array of q_start
        
        q_goal=numpy.array(q_goal) # Numpy array of q_goal
        
        q_tree={0:[q_start,0]}    # This tree inlcudes the node number as a key and postion and parent node number as a value
        
        points=[q_start]  # List with all of the points in our tree

        step=0.5  # Step between each node
        
        reached_goal=False
        
        max_nodes=200
        
        now=rospy.get_rostime().secs
        
        begin=rospy.get_rostime().secs
        # Create the loop through which the tree will be built
        
        
        '''
# and ((len(q_tree) < max_nodes) or ((now - begin) < 30))
		'''
        while (not reached_goal) and ((len(q_tree) < max_nodes) or ((now - begin) < 30)):
            # Create random configuration space vector
            q_rand=[]
            for idx in range(len(q_min)):
                q_rand.append(q_min[idx]+(random.random()*(q_max[idx]-q_min[idx])))
            
            q_rand=numpy.array(q_rand)
            
            # Find the point in tree with the minimum distance from the random point
            min_key=0
            min_distance=numpy.linalg.norm(q_rand-q_tree[min_key][0])
            
            tree_keys=list(q_tree.keys())
            tree_keys.sort()
            
            for key in tree_keys:
                if numpy.linalg.norm(q_rand-q_tree[key][0]) < min_distance:
                    min_key=key   # Key of minimum point
            
            # find branch of predefined length in the direction of the random point.
            V_direction = (q_rand-q_tree[min_key][0])/numpy.linalg.norm(q_rand-q_tree[min_key][0])
            
            new_point= q_tree[min_key][0]+step*V_direction
            
            # Check if the branch is intersecting with the obstacles
            
            for disc_step in numpy.linspace(0.0,step,num=5):
                
                state=True 
                
                V_direction = (new_point-q_tree[min_key][0])/numpy.linalg.norm(new_point-q_tree[min_key][0])
                
                desc_point= q_tree[min_key][0]+disc_step*V_direction
                
                if self.is_state_valid(desc_point) == False:
                    state=False
                    break
                    
            
            if state == False:
                continue
                
            # add branch to the tree
            key=len(q_tree)
            q_tree[key]=[new_point,min_key]
            points.append(new_point)
            now=rospy.get_rostime().secs
            # Check if there is a clear path between the new point and the goal point
            p_g_V=q_goal-new_point
            
            direction=p_g_V/numpy.linalg.norm(p_g_V)
            
            magnitude=numpy.linalg.norm(p_g_V)
            
            for desc_step in numpy.linspace(0,magnitude,round(magnitude/0.2)):
                state=True
                desc_point=new_point+ desc_step*direction
                
                if self.is_state_valid(desc_point) == False:
                    state=False
                    break
            
            if state == False:
                continue
                
            else:
                q_tree[len(q_tree)]=[q_goal,key]
                points.append(q_goal)
                reached_goal=True
        print('-----')
        print(q_tree)
        
        
        # Trace the path back from the goal to the starting point
        q_list=[list(q_goal)]
        
        current_node=len(points)-1
        parent=q_tree[current_node][1]
        
        path_is_complete=False
        
        while not path_is_complete:
            # append the parent point
            q_list.append(list(q_tree[parent][0]))
            parent=q_tree[parent][1]
            
            if parent == 0:
                q_list.append(list(q_start))
                path_is_complete=True
        
        q_list.reverse()   
               
        print(q_start)
        print('_______')
        print(q_goal)
        print('_______')
        print(q_list)
             
        
        ###########################################################
        # Create a random configuration space vector
        # size of this vector = 7 (number of joints)
        
        
        ###################################################
       

        return q_list
    
    
    def create_trajectory(self, q_list, v_list, a_list, t):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            point.velocities = list(v_list[i])
            point.accelerations = list(a_list[i])
            point.time_from_start = rospy.Duration(t[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def create_trajectory(self, q_list):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def project_plan(self, q_start, q_goal, q_min, q_max):
        q_list = self.motion_plan(q_start, q_goal, q_min, q_max)
        joint_trajectory = self.create_trajectory(q_list)
        return joint_trajectory

    def move_arm_cb(self, msg):
        T = convert_from_trans_message(msg)
        self.mutex.acquire()
        q_start = self.q_from_joint_state(self.joint_state)
        print "Solving IK"
        q_goal = self.IK(T)
        if len(q_goal)==0:
            print "IK failed, aborting"
            self.mutex.release()
            return
        print "IK solved, planning"
        trajectory = self.project_plan(numpy.array(q_start), q_goal, self.q_min, self.q_max)
        if not trajectory.points:
            print "Motion plan failed, aborting"
        else:
            print "Trajectory received with " + str(len(trajectory.points)) + " points"
            self.execute(trajectory)
        self.mutex.release()
        
    def joint_states_callback(self, joint_state):
        self.mutex.acquire()
        self.joint_state = joint_state
        self.mutex.release()

    def execute(self, joint_trajectory):
        self.pub_trajectory.publish(joint_trajectory)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()