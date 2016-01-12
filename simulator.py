import rospy, rospkg
rospack = rospkg.RosPack()
import sensor_msgs.msg as sm

import openravepy as rave
import numpy as np

import os.path
import time

import IPython

class Simulator:
    """ OpenRave simulator """
    def __init__(self, env_file=None, view=False):
        if env_file is None:
            env_file = rospack.get_path('pr2_utils') + '/robots/my-pr2-beta-sim.robot.xml'
        self.joint_state_msg = None
        self.joint_state_sub = rospy.Subscriber('/joint_states', sm.JointState, self._joint_state_callback)
        
        self.env = rave.Environment()
        self.env.StopSimulation()
        self.env.Load(env_file)
        
        self.added_kinbody_names = list()
        
        self.handles = list()
        self.view = view
        if view:
            self.env.SetViewer('qtcoin')
        
        self.robot = self.env.GetRobots()[0]
        self.larm = self.robot.GetManipulator('leftarm')
        self.rarm = self.robot.GetManipulator('rightarm')
        
        for arm in [self.larm, self.rarm]:
            self.robot.SetActiveManipulator(arm)
        
            ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=rave.IkParameterizationType.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
        
    def update(self):
        """
        Updates robot joints to match ROS /joint_states
        """
        msg = self.joint_state_msg
        if msg is None:
            print('Cannot update simulator, no joint state message received')
            return
        
        indices, joint_values = list(), list()
        for name, joint_value in zip(msg.name, msg.position):
            for joint in self.robot.GetJoints():
                if joint.GetName() == name:
                    indices.append(joint.GetDOFIndex())
                    joint_values.append(joint_value)
                    break
        
        lower_limits, upper_limits = self.robot.GetDOFLimits(indices)
        joint_values = np.maximum(joint_values, lower_limits)
        joint_values = np.minimum(joint_values, upper_limits)
                
        self.robot.SetDOFValues(joint_values, indices)
        
    def _joint_state_callback(self, msg):
        """
        :type msg: sensor_msgs/JointState
        """
        self.joint_state_msg = msg
        
    #################
    # frame methods #
    #################
        
    def transform_from_to(self, p, ref_link_name, targ_link_name):
        """
        :param p: 4x4 or 3d np.array
        :param ref_link_name: string
        :param targ_link_name: string
        """
        p = np.array(p)
        shape = p.shape
        if len(shape) == 1 and shape[0] == 3:
            pose_mat_in_ref = np.eye(4)
            pose_mat_in_ref[:3,3] = p
            is_position = True
        elif len(shape) == 2 and shape == (4,4):
            pose_mat_in_ref = p
            is_position = False
        else:
            print('Incorrect array size in transform_from_to')
            return p
        
        ref_link_name = ref_link_name.replace('/','')
        targ_link_name = targ_link_name.replace('/','')
        
        # ref -> world
        if ref_link_name != 'world':
            ref_from_world = self.robot.GetLink(ref_link_name).GetTransform()
        else:
            ref_from_world = np.eye(4)
    
        # target -> world
        if targ_link_name != 'world':
            targ_from_world = self.robot.GetLink(targ_link_name).GetTransform()
        else:
            targ_from_world = np.eye(4)
    
        # target -> ref
        targ_from_ref = np.dot(np.linalg.inv(targ_from_world), ref_from_world)
    
        pose_mat_in_targ = np.array(np.dot(targ_from_ref, pose_mat_in_ref))
        
        if is_position:
            return pose_mat_in_targ[:3,3]
        else:
            return pose_mat_in_targ
    
    def transform_relative_pose_for_ik(self, manip, matrix4, ref_frame, targ_frame):
        """
        Transforms the matrix to be used for IK
        (needed since last link in manipulator is not tool_frame
        
        :param manip: OpenRave manipulator
        :param matrix4: 4x4 np.array
        :param ref_frame: string reference frame
        :param targ_frame: string target frame
        """
        if ref_frame == 'world':
            world_from_ref = np.eye(4)
        else:
            ref = self.robot.GetLink(ref_frame)
            world_from_ref = ref.GetTransform()
    
        if targ_frame == 'end_effector':        
            targ_from_EE = np.eye(4)
        else:
            world_from_targ = self.robot.GetLink(targ_frame).GetTransform()
            world_from_EE = manip.GetEndEffectorTransform()    
            targ_from_EE = np.dot(np.linalg.inv(world_from_targ), world_from_EE)       
    
        ref_from_targ_new = matrix4
        world_from_EE_new = np.dot(np.dot(world_from_ref, ref_from_targ_new), targ_from_EE)    
    
        return np.array(world_from_EE_new)
    
    def ik_for_link(self, pose_mat, manip, link_name, filter_options=18, return_all_solns=False):
        """
        Perform IK for an arbitrary link attached to the manipulator
        e.g. you might want ik for pr2 "r_gripper_tool_frame" instead of the openrave EE frame
        
        :param pose_mat: 4x4 matrix "world frame from link frame"
        :param manip: OpenRAVE manipulator
        :param link_name: frame of pose_mat
        :param filter_options: see openravepy.IkFilterOptions
        :param return_all_solns: if True, returns a list. if false, returns a single solution (OpenRAVE's default)
        :return if a solution exists, otherwise return None
        """
        pose_mat = np.array(pose_mat)
        link = self.robot.GetLink(link_name)
    
        if not self.robot.DoesAffect(manip.GetArmJoints()[-1], link.GetIndex()):
            print("Link {0} is not attached to end effector of manipulator {1}".format(link_name, manip.GetName()))
            return
    
        link_pose_mat = link.GetTransform()
        ee_pose_mat = manip.GetEndEffectorTransform()
        link_to_ee = np.linalg.solve(link_pose_mat, ee_pose_mat)
        
        ee_from_world = pose_mat.dot(link_to_ee)
        
        self.robot.SetActiveManipulator(manip)
        ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=rave.IkParameterizationType.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
            
        if return_all_solns:
            return ikmodel.manip.FindIKSolutions(ee_from_world, filter_options)
        else:
            return ikmodel.manip.FindIKSolution(ee_from_world, filter_options)
        
    ######################
    # update environment #
    ######################
    
    def clear_kinbodies(self):
        """
        Removes all user added kinbodies
        """
        for name in self.added_kinbody_names:
            self.env.Remove(self.env.GetKinBody(name))
        self.added_kinbody_names = list()
        
    def remove_colliding_kinbodies(self):
        new_added_kinbody_names = list()
        for name in self.added_kinbody_names:
            body = self.env.GetKinBody(name)
            if self.env.CheckCollision(self.robot, body):
                self.env.Remove(body)
            else:
                new_added_kinbody_names.append(name)
                
        self.added_kinbody_names = new_added_kinbody_names
    
    def add_kinbody(self, vertices, triangles, name=None, check_collision=False):
        """
        :param vertices: list of 3d np.ndarray corresponding to points in the mesh
        :param triangles: list of 3d indices corresponding to vertices
        :param name: name of the kinbody to be added
        :param check_collision: if True, will not add kinbody if it collides with the robot
        :return False if check_collision=True and there is a collision
        """
        name = name if name is not None else 'kinbody'+str(time.time())
        self.added_kinbody_names.append(name)
        
        body = rave.RaveCreateKinBody(self.env, "")
        body.InitFromTrimesh(trimesh=rave.TriMesh(vertices, triangles), draw=True) 
        body.SetName(name) 
        self.env.Add(body)
        
        randcolor = np.random.rand(3)
        body.GetLinks()[0].GetGeometries()[0].SetAmbientColor(randcolor)
        body.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(randcolor)
        
        if check_collision:
            if self.env.CheckCollision(self.robot, body):
                self.env.Remove(body)
                self.added_kinbody_names = self.added_kinbody_names[:-1]
                return False
            
        return True
                
    def add_box(self, pose, extents, name=None, check_collision=False):
        """
        :param pose: 4x4 np.ndarray in frame world
        :param extents: length 3 list/np.ndarray of axis lengths
        :param name: name of kinbod to be added
        :param check_collision: if True, will not add kinbody if it collides with the robot
        :return False if check_collision=True and there is a collision
        """
        name = name if name is not None else 'kinbody'+str(time.time())
        pose = np.array(pose)
        rot = pose[:3,:3]
        trans = pose[:3,3]
        
        box = rave.RaveCreateKinBody(self.env, '')
        rx, ry, rz = list(extents)
        vertices = np.array([
            [-rx, -ry, -rz],
            [-rx, -ry, rz],
            [-rx, ry, -rz],
            [-rx, ry, rz],
            [rx, -ry, -rz],
            [rx, -ry, rz],
            [rx, ry, -rz],
            [rx, ry, rz]])
        vertices = [rot.dot(v)+trans for v in vertices]
        triangles = [
            [0,1,2],
            [3,1,2],
            [0,1,4],
            [5,1,4],
            [0,2,4],
            [6,2,4],
            [7,6,5],
            [4,6,5],
            [7,6,3],
            [2,6,3],
            [7,5,3],
            [1,5,3]]
        return self.add_kinbody(vertices, triangles, name=name, check_collision=check_collision)
    
    ############
    # plotting #
    ############
    
    def clear_plots(self, num_to_clear=-1):
        """
        :param num_to_clear: if num_to_clear < 0, clear all plots, else clear num_to_clear
        """
        if num_to_clear < 0:
            self.handles = list()
        else:
            self.handles = self.handles[:-int(min(num_to_clear, len(self.handles)))]
    
    def plot_point(self, pos_array, size=.01, color=(0,1,0)):
        """
        :param pos_array: 3d np.array
        :param size: radius in meters
        :param color: rgb [0,1], default green
        """
        self.handles += [self.env.plot3(points=pos_array,
                                        pointsize=size,
                                        colors=np.array(color),
                                        drawstyle=1)]
    
    def plot_segment(self, start, end, color=(1,0,0)):
        """
        :param start: 3d np.array
        :param end: 3d np.array
        :param color: rgb [0,1], default red
        """
        start = np.array(start)
        end = np.array(end)
        
        self.handles += [self.env.drawlinestrip(points=np.array([start, end]), linewidth=3.0, colors=np.array([color,color]))]
        
    def plot_triangle(self, points, color=(1,0,0), alpha=1.):
        """
        :param points: length 3 list of 3d list/np.array
        :param color: (r,g,b) [0,1]
        :param alpha: [0,1]
        """
        self.handles += [self.env.drawtrimesh(points=np.vstack(points),
                                              indices=None,
                                              colors=np.array(color+(alpha,)))]
    
    def plot_transform(self, T, s=0.1):
        """
        :param T: 4x4 np.array
        :param s: length of axes in meters
        """
        T = np.array(T)
        h = []
        x = T[0:3,0]
        y = T[0:3,1]
        z = T[0:3,2]
        o = T[0:3,3]
        self.handles.append(self.env.drawlinestrip(points=np.array([o, o+s*x]), linewidth=3.0, colors=np.array([(1,0,0),(1,0,0)])))
        self.handles.append(self.env.drawlinestrip(points=np.array([o, o+s*y]), linewidth=3.0, colors=np.array(((0,1,0),(0,1,0)))))
        self.handles.append(self.env.drawlinestrip(points=np.array([o, o+s*z]), linewidth=3.0, colors=np.array(((0,0,1),(0,0,1)))))
    
    def save_view(self, file_name):
        """
        :param file_name: path string
        """
        self.env.GetViewer().SendCommand('SetFiguresInCamera 1') # also shows the figures in the image
        I = self.env.GetViewer().GetCameraImage(640,480,  self.env.GetViewer().GetCameraTransform(),[640,640,320,240])
        scipy.misc.imsave(file_name ,I)
        self.env.GetViewer().SendCommand('SetFiguresInCamera 0')
       