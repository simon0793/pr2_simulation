import openravepy as rave
import trajoptpy
import trajoptpy.kin_utils as ku
from trajoptpy.check_traj import traj_is_safe, traj_collisions
import trajoptpy.math_utils as mu
import json

import rospy
import roslib
roslib.load_manifest('tfx')
import tfx

import numpy as np

import simulator
import utils

import IPython

class Planner:
    def __init__(self, arm_name, sim=None, interact=False):
        """
        :param arm_name: "left" or "right"
        :param sim: OpenRave simulator (or create if None)
        :param interact: enable trajopt viewer
        """
        assert arm_name == 'left' or arm_name == 'right'
        
        self.sim = sim
        if self.sim is None:
            self.sim = simulator.Simulator()
            
        self.robot = self.sim.robot
        self.manip = self.sim.larm if arm_name == 'left' else self.sim.rarm
        
        wrist_flex_index = self.robot.GetJointIndex(arm_name[0]+'_wrist_flex_joint')
        lower, upper = self.robot.GetDOFLimits()
        lower[wrist_flex_index], upper[wrist_flex_index] = -np.pi/2., np.pi/2.
        self.robot.SetDOFLimits(lower, upper)
        
        if interact:
            trajoptpy.SetInteractive(True)
        
        self.tool_frame = '{0}_gripper_tool_frame'.format(arm_name[0])
        self.joint_indices = self.manip.GetArmIndices()
        
    def get_grasp_joint_trajectory(self, start_joints, target_pose, n_steps=40, ignore_orientation=False, link_name=None):
        """
        Calls trajopt to plan grasp collision-free trajectory
        
        :param start_joints: list of initial joints
        :param target_pose: desired pose of tool_frame (tfx.pose)
        :param n_steps: trajopt discretization
        :return None if traj not collision free, else list of joint values
        """
        link_name = link_name if link_name is not None else self.tool_frame
        
        assert len(start_joints) == len(self.joint_indices)
        assert target_pose.frame.count('base_link') == 1
        self.sim.update()
        
        # set active manipulator and start joint positions
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        
        # initialize trajopt inputs
        rave_pose = tfx.pose(self.sim.transform_from_to(target_pose.matrix, target_pose.frame, 'world'))
        quat = rave_pose.orientation
        xyz = rave_pose.position
        quat_target = [quat.w, quat.x, quat.y, quat.z]
        xyz_target = [xyz.x, xyz.y, xyz.z]
        rave_mat = rave.matrixFromPose(np.r_[quat_target, xyz_target])
        
#         init_joint_target = None
        init_joint_target = self.sim.ik_for_link(rave_pose.matrix, self.manip, link_name, 0)
        if init_joint_target is not None:
            init_joint_target = self._closer_joint_angles(init_joint_target, start_joints)
        
        init_traj = self.ik_point(start_joints, xyz, n_steps=n_steps, link_name=link_name)
        
        request = self._get_grasp_trajopt_request(xyz_target, quat_target, n_steps,
                                                  ignore_orientation=ignore_orientation, link_name=link_name, init_traj=init_traj)
        
        # convert dictionary into json-formatted string
        s = json.dumps(request) 
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.sim.env)
            
        # TODO: worth doing?
#         tool_link = self.robot.GetLink(link_name)
#         def point_at(x):
#             self.robot.SetDOFValues(x, self.joint_indices, False)
#             T = tool_link.GetTransform()
#             local_dir = xyz.array - T[:3,3]
#             return T[1:3,:3].dot(local_dir)
#           
#         for t in xrange(int(0.8*n_steps), n_steps-1):
#             #prob.AddConstraint(point_at, [(t,j) for j in xrange(len(self.joint_indices))], "EQ", "POINT_AT_%i"%t)
#             prob.AddErrorCost(point_at, [(t,j) for j in xrange(len(self.joint_indices))], "ABS", "POINT_AT_%i"%t)

        # do optimization
        result = trajoptpy.OptimizeProblem(prob)
        
        prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        #num_upsampled_collisions = len(traj_collisions(result.GetTraj(), self.robot, n=100))
        num_upsampled_collisions = self._num_collisions(result.GetTraj())
        print('Number of collisions: {0}'.format(num_upsampled_collisions))
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        if num_upsampled_collisions > 2:
        #if not traj_is_safe(result.GetTraj()[:], self.robot): # Check that trajectory is collision free
            return None
        else:
            return result.GetTraj()
        
    def _get_grasp_trajopt_request(self, xyz_target, quat_target, n_steps, ignore_orientation=False, link_name=None, init_traj=None):
        """
        :param xyz_target: 3d list
        :param quat_target: [w,x,y,z]
        :param n_steps: trajopt discretization
        :param ignore_orientation
        :param init_traj: if not None, traj initialization of target_pose for trajopt
        :return trajopt json request
        """
        link_name = link_name if link_name is not None else self.tool_frame
        rot_coeffs = [1,1,1] if not ignore_orientation else [0,0,0]
        
        request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : str(self.manip.GetName()), 
                "start_fixed" : True 
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]} 
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20], # 20
                        "continuous": False,
                        "dist_pen" : [0.05] # .025 
                        }
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [40], # 20
                        "continuous" : True,
                        "dist_pen" : [0.05] # .025 
                        }
                    },
                {
                        "type" : "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : xyz_target, 
                                    "wxyz" : quat_target,
                                    "link": link_name,
                                    "rot_coeffs" : rot_coeffs,
                                    "pos_coeffs" : [0,0,0],
                                    }
                        },
                ],
            "constraints" : [
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,
                                "link": link_name,
                                "rot_coeffs" : [0,0,0],
                                "pos_coeffs" : [1,1,1]
                                }
                     
                    },
                {
                    "type" : "cart_vel",
                    "name" : "cart_vel",
                    "params" : {
                                "max_displacement" : .02,
                                "first_step" : 0,
                                "last_step" : n_steps-1, #inclusive
                                "link" : link_name
                                },
                 }
                ],
            }
        
        if init_traj is None:
            request["init_info"] = {
                                     "type" : "stationary",
                                     }
        else:
            request["init_info"] = {
                                     "type" : "given_traj",
                                     "data" : init_traj.tolist(),
                                     }
        
        
        return request
    
    def get_return_from_grasp_joint_trajectory(self, start_joints, target_pose, n_steps=40):
        """
        Calls trajopt to plan return from grasp collision-free trajectory
        
        :param start_joints: list of initial joints
        :param target_pose: desired pose of tool_frame (tfx.pose)
        :param n_steps: trajopt discretization
        :return None if traj not collision free, else list of joint values
        """
        assert len(start_joints) == len(self.joint_indices)
        assert target_pose.frame.count('base_link') == 1
        
        # set active manipulator and start joint positions
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        
        # initialize trajopt inputs
        rave_pose = tfx.pose(self.sim.transform_from_to(target_pose.matrix, target_pose.frame, 'world'))
        quat = rave_pose.orientation
        xyz = rave_pose.position
        quat_target = [quat.w, quat.x, quat.y, quat.z]
        xyz_target = [xyz.x, xyz.y, xyz.z]
        rave_mat = rave.matrixFromPose(np.r_[quat_target, xyz_target])
        
        request = self._get_return_from_grasp_trajopt_request(xyz_target, quat_target, n_steps)
        
        # convert dictionary into json-formatted string
        s = json.dumps(request) 
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.sim.env)
        
        tool_link = self.robot.GetLink(self.tool_frame)
        def penalize_low_height(x):
            self.robot.SetDOFValues(x, self.joint_indices, False)
            z = tool_link.GetTransform()[2,3]
            return max(0, 10.0 - z)

        for t in xrange(n_steps-2):
            prob.AddErrorCost(penalize_low_height, [(t,j) for j in xrange(len(self.joint_indices))], "ABS", "PENALIZE_LOW_HEIGHT_%i"%t)
                        
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)
        
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        num_upsampled_collisions = self._num_collisions(result.GetTraj())
        print('Number of collisions: {0}'.format(num_upsampled_collisions))
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        if num_upsampled_collisions > 2:
            return None
        else:
            return result.GetTraj()
        
    def _get_return_from_grasp_trajopt_request(self, xyz_target, quat_target, n_steps):
        """
        :param xyz_target: 3d list
        :param quat_target: [w,x,y,z]
        :param n_steps: trajopt discretization
        :return trajopt json request
        """
        request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : str(self.manip.GetName()), 
                "start_fixed" : True 
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]} 
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20],
                        "continuous": False,
                        "dist_pen" : [0.025] 
                        }
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20],
                        "continuous" : True,
                        "dist_pen" : [0.025] 
                        }
                    },
#                 {
#                         "type" : "pose",
#                         "name" : "target_pose",
#                         "params" : {"xyz" : xyz_target, 
#                                     "wxyz" : quat_target,
#                                     "link": self.tool_frame,
#                                     "rot_coeffs" : [1,1,1],
#                                     "pos_coeffs" : [0,0,0],
#                                     }
#                         },
                ],
            "constraints" : [
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,
                                "link": self.tool_frame,
                                "rot_coeffs" : [1,1,1],
                                "pos_coeffs" : [1,1,1]
                                }
                     
                    },
                 {
                    "type" : "cart_vel",
                    "name" : "cart_vel",
                    "params" : {
                                "max_displacement" : .02,
                                "first_step" : 0,
                                "last_step" : n_steps-1, #inclusive
                                "link" : self.tool_frame
                                },
                 }
                ],
            "init_info" : {
                            "type" : "stationary",
                    },
            }
        
        return request
    
    def ik_point(self, start_joints, target_position, n_steps=40, link_name=None):
        """
        Calls trajopt to get traj to point, no collision checking
        
        :param start_joints: list of initial joints
        :param target_position: 3d np.ndarray desired position of tool_frame in world frame
        :param n_steps: trajopt discretization
        :return None if traj not collision free, else list of joint values
        """
        link_name = link_name if link_name is not None else self.tool_frame
        
        assert len(start_joints) == len(self.joint_indices)
        self.sim.update()
        
        # set active manipulator and start joint positions
        self.robot.SetDOFValues(start_joints, self.joint_indices)
        
        request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : str(self.manip.GetName()), 
                "start_fixed" : True 
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]} 
                    },
                ],
            "constraints" : [
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : list(target_position), 
                                "wxyz" : [0,0,0,1],
                                "link": link_name,
                                "rot_coeffs" : [0,0,0],
                                "pos_coeffs" : [1,1,1]
                                }
                     
                    },
                ],
            "init_info" : {
                            "type" : "stationary",
                    },
            }
        
        # convert dictionary into json-formatted string
        s = json.dumps(request) 
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.sim.env)
        
        tool_link = self.robot.GetLink(self.tool_frame)
        def penalize_low_height(x):
            self.robot.SetDOFValues(x, self.joint_indices, False)
            z = tool_link.GetTransform()[2,3]
            return max(0, 10.0 - z)

        for t in xrange(n_steps-2):
            prob.AddErrorCost(penalize_low_height, [(t,j) for j in xrange(len(self.joint_indices))], "ABS", "PENALIZE_LOW_HEIGHT_%i"%t)
        
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)
        
        return result.GetTraj()
    
    @staticmethod
    def _closer_joint_angles(new_joints, curr_joints):
        for i in [2, 4, 6]:
            new_joints[i] = utils.closer_angle(new_joints[i], curr_joints[i])
        return new_joints
        
    def _num_collisions(self, joint_traj, up_samples=120):
        traj_up = mu.interp2d(np.linspace(0,1,up_samples), np.linspace(0,1,len(joint_traj)), joint_traj)
        
        manip_links = [l for l in self.robot.GetLinks() if l not in self.manip.GetIndependentLinks()]
        
        num_collisions = 0
        for (i,row) in enumerate(traj_up):
            self.robot.SetDOFValues(row, self.joint_indices)
            #is_collision = max([self.sim.env.CheckCollision(l) for l in manip_links])
            is_collision = self.sim.env.CheckCollision(self.robot)
            if is_collision:
                num_collisions += 1
                
        return num_collisions
        
 