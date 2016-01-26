import rospy, roslib
roslib.load_manifest('tfx')
import tfx

import geometry_msgs.msg as gm
import trajectory_msgs.msg as tm

import numpy as np

import arm
import simulator

import IPython

class TestPR2Grasp:
    def __init__(self):
        #self.grasp_traj = None
        #self.return_grasp_traj = None
        
        #self.grasp_traj_sub = rospy.Subscriber('/check_handle_grasps/grasp_trajectory', gm.PoseArray, self._grasp_traj_callback)
        #self.return_grasp_traj_sub = rospy.Subscriber('/check_handle_grasps/return_grasp_trajectory', gm.PoseArray, self._return_grasp_traj_callback)
        
        self.grasp_joint_traj = None
        self.return_grasp_joint_traj = None
        
        self.grasp_joint_traj_sub = rospy.Subscriber('/check_handle_grasp/grasp_joint_trajectory', tm.JointTrajectory, self._grasp_joint_traj_callback)
        self.return_grasp_joint_traj_sub = rospy.Subscriber('/check_handle_grasp/return_grasp_joint_trajectory', tm.JointTrajectory, self._return_grasp_joint_traj_callback)
        
        self.sim = simulator.Simulator(view=True)
        self.arm = arm.Arm('right', sim=self.sim)
        self.speed = 0.06
        
#     def _grasp_traj_callback(self, msg):
#         self.grasp_traj = [tfx.pose(p, frame='base_link') for p in msg.poses]
#         
#     def _return_grasp_traj_callback(self, msg):
#         self.return_grasp_traj = [tfx.pose(p, frame='base_link') for p in msg.poses]
        
    def _grasp_joint_traj_callback(self, msg):
        self.grasp_joint_traj = [jt_pt.positions for jt_pt in msg.points]
        
    def _return_grasp_joint_traj_callback(self, msg):
        self.return_grasp_joint_traj = [jt_pt.positions for jt_pt in msg.points]
        
    def run(self):
        while not rospy.is_shutdown():
            self.grasp_joint_traj = None
            self.return_grasp_joint_traj = None
            
            print('Waiting for grasp trajectories')
            while not rospy.is_shutdown() and (self.grasp_joint_traj is None or self.return_grasp_joint_traj is None):
                self.sim.update()
                rospy.sleep(1)
            
            print('Grasping!')
            grasp_joint_traj = self.grasp_joint_traj
            return_grasp_joint_traj = self.return_grasp_joint_traj
    
            print('Opening gripper')        
            self.arm.open_gripper()
            rospy.sleep(1)
            print('Going to grasp')
            self.arm.execute_joint_trajectory(grasp_joint_traj, speed=0.06)
            print('Closing grippper')
            self.arm.close_gripper()
            rospy.sleep(1.5)
            print('Returning from grasp')
            self.arm.execute_joint_trajectory(return_grasp_joint_traj, speed=0.06)
            print('Opening gripper')
            self.arm.open_gripper()

if __name__ == '__main__':
    rospy.init_node('test_pr2_grasp', anonymous=True)
    rospy.sleep(1)
    
    test_grasp = TestPR2Grasp()
    test_grasp.run()