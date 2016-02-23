import arm
import planner

import numpy as np

import rospy
import roslib
import actionlib
roslib.load_manifest("pr2_controllers_msgs")
roslib.load_manifest("move_base_msgs")
#from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
import trajectory_msgs.msg as tm
import sensor_msgs.msg as sm
import pr2_controllers_msgs.msg as pcm
#import control_msgs.msg as pcm


roslib.load_manifest('tfx')
import tfx
import tf.transformations as tft

import openravepy as rave

'''EXPLICIT LOCATIONS USED'''
RED = [0.701716, 0.400784, 0.83095]
BLUE = [0.701716, 0.000784, 0.83095]
GREEN = [0.701716, -0.400784, 0.83095]
RIGHT_UP = [0.5, -0.2, 1.25]
LEFT_UP = [0.5, 0.2, 1.25]
TARGET_GREEN = [0.6, -0.405, 0.77]
GRASP_GREEN = [0.7, -0.405, 0.77]
DEST_GREEN = [0.7, 0.0, 0.79]


class SimpleArmState:
  ''' Class for representing just the state of PR2 arms (no additional objects)
  '''
  def __init__(self):
    '''
    '''
    self.rightArm = arm.Arm('right')
    self.leftArm = arm.Arm('left')
    self.rightArm.sim.env.SetViewer('qtcoin')
    self.valid = True
    self.graspPlanner = planner.Planner('right')
    # self.leftArm.sim.env.SetViewer('qtcoin')

  def isGoalState(self):
    ''' Returns true when goal state is reached
      by default always returns true
    '''

    ''' ------ CHANGE CODE BELOW THIS POINT ------ '''
    # 0.630, -0.740,  0.775
    # 0.632,  0.077,  0.772
    rightArmPose = self.rightArm.get_pose()
    targetX, targetY, targetZ = DEST_GREEN
    currentX, currentY, currentZ = rightArmPose.position.array
    acceptableAbsError = 0.1
    rightValid = abs(targetX - currentX) <= acceptableAbsError and abs(targetY - currentY) <= acceptableAbsError and abs(targetZ - currentZ) <= acceptableAbsError # Modify to check the current state
    leftArmPose = self.leftArm.get_pose()
    targetX, targetY, targetZ = DEST_GREEN
    currentX, currentY, currentZ = leftArmPose.position.array
    acceptableAbsError = 0.05
    leftValid = abs(targetX - currentX) <= acceptableAbsError and abs(targetY - currentY) <= acceptableAbsError and abs(targetZ - currentZ) <= acceptableAbsError # Modify to check the current state
    return rightValid

    ''' ------ CHANGE CODE ABOVE THIS POINT ------ '''


  def successorState(self):
    ''' Updates current state to the next successor state
      by default it just returns current state
    '''
    rightArmPose = self.rightArm.get_pose()
    leftArmPose = self.leftArm.get_pose()
    newRightArmPose = tfx.pose(rightArmPose) # Create new pose
    newLeftArmPose = tfx.pose(leftArmPose) # Create new pose

    ''' ------ CHANGE CODE BELOW THIS POINT ------ '''
    if self.valid:
      x1, y1, z1 = TARGET_GREEN
      x2, y2, z2 = GRASP_GREEN
      currentX, currentY, currentZ = rightArmPose.position.array
      acceptableAbsError = 0.1
      if abs(x1 - currentX) <= acceptableAbsError and abs(y1 - currentY) <= acceptableAbsError and abs(z1 - currentZ) <= acceptableAbsError:
        newRightArmPose.position = GRASP_GREEN
      elif abs(x2 - currentX) <= acceptableAbsError and abs(y2 - currentY) <= acceptableAbsError and abs(z2 - currentZ) <= acceptableAbsError:
        self.rightArm.close_gripper()
        print "Grasping..."
        rospy.sleep(1)
        self.raiseArms()
        newRightArmPose.position = DEST_GREEN
      else:
        print "ERROR1: " + str([abs(x1 - currentX), abs(y1 - currentY), abs(z1 - currentZ)])
        print "ERROR2: " + str([abs(x2 - currentX), abs(y2 - currentY), abs(z2 - currentZ)])
        rospy.sleep(1)
        newRightArmPose.position = TARGET_GREEN # either update by adding to xyz offset array "+ [0, -pos_step, 0]" or assign new position, defualt keeps position the same

    else:
      print "The last successor state was invalid"
      print "Enter a custom pose x y z"
      coord = raw_input().split()
      newRightArmPose.position = [float(x) for x in coord]

    newLeftArmPose.position = leftArmPose.position

    ''' ------ CHANGE CODE ABOVE THIS POINT ------ '''

    # newRightArmPose = tfx.pose(newRightArmPose.position, tfx.tb_angles(0.0, 0.0, 0.0))
    newRightJointsList = self.graspPlanner.get_grasp_joint_trajectory(self.rightArm.get_joints(), newRightArmPose.position, n_steps=10)
    print newRightJointsList
    self.rightArm.execute_joint_trajectory(newRightJointsList)
    # for newRightJoints in newRightJointsList:
    #   if newRightJoints is not None:
    #     self.rightArm.go_to_joints(newRightJoints)
    #     print('new_pose: {0}'.format(newRightArmPose))
    #     self.valid = True
    #   else:
    #     rospy.loginfo('Invalid Right Arm Pose')
    #     print newRightArmPose
    #     self.valid = False
    #     return self
    #   rospy.sleep(1)

    # newRightJoints = self.rightArm.ik(newRightArmPose) # Updates arm pose if valid
    # newLeftJoints = self.leftArm.ik(newLeftArmPose)
    # if newRightJoints is not None:
    #   self.rightArm.go_to_joints(newRightJoints)
    #   print('new_pose: {0}'.format(newRightArmPose))
    #   self.valid = True
    # else:
    #   rospy.loginfo('Invalid Right Arm Pose')
    #   print newRightArmPose
    #   self.valid = False
    #   return self
    # rospy.sleep(.01)
    # if newLeftJoints is not None:
    #   self.leftArm.go_to_joints(newLeftJoints)
    # else:
    #   rospy.loginfo('Invalid Left Arm Pose')
    rospy.sleep(.01)
    return self # Modify to generate a valid successor states, does not update state if pose is invalid and instead reverts to last valid state

  def alignGrippers(self):
    rightArmPose = self.rightArm.get_pose()
    leftArmPose = self.leftArm.get_pose()
    newLeftArmPose = tfx.pose(leftArmPose.position, tfx.tb_angles(0.0, 0.0, 0.0)) # Create new pose
    # print "yaw(current = " + str(rightArmPose.tb_angles.yaw_deg) + "):"
    # coord = raw_input()
    # yaw = float(coord)
    # print "pitch(current = " + str(rightArmPose.tb_angles.pitch_deg) + "):"
    # coord = raw_input()
    # pitch = float(coord)
    # print "roll(current = " + str(rightArmPose.tb_angles.roll_deg) + "):"
    # coord = raw_input()
    # roll = float(coord)
    newRightArmPose = tfx.pose(rightArmPose.position, tfx.tb_angles(0.0, 0.0, 0.0))
    newRightJoints = self.rightArm.ik(newRightArmPose) # Updates arm pose if valid
    newLeftJoints = self.leftArm.ik(newLeftArmPose)
    if newRightJoints is not None:
      self.rightArm.go_to_joints(newRightJoints)
      print('new_pose: {0}'.format(newRightArmPose))
      print "Aligning right gripper..."
      self.rightArm.open_gripper()
      self.valid = True
    else:
      rospy.loginfo('Invalid Right Arm Pose')
      print "RIGHT ALIGNMENT FAILED"
      self.valid = False
      return self
    rospy.sleep(0.5)
    if newLeftJoints is not None:
      self.leftArm.go_to_joints(newLeftJoints)
      print "Aligning left gripper..."
      self.leftArm.open_gripper()
    else:
      rospy.loginfo('Invalid Left Arm Pose')
      print "LEFT ALIGNMENT FAILED"
    rospy.sleep(0.5)
    return self

  def raiseArms(self):
    rightArmPose = self.rightArm.get_pose()
    leftArmPose = self.leftArm.get_pose()
    newRightArmPose = tfx.pose(rightArmPose) # Create new pose
    newLeftArmPose = tfx.pose(leftArmPose) # Create new pose
    newRightArmPose.position = RIGHT_UP # either update by adding to xyz offset array "+ [0, -pos_step, 0]" or assign new position, defualt keeps position the same
    newLeftArmPose.position = LEFT_UP
    newRightJoints = self.rightArm.ik(newRightArmPose) # Updates arm pose if valid
    newLeftJoints = self.leftArm.ik(newLeftArmPose)
    if newRightJoints is not None:
      self.rightArm.go_to_joints(newRightJoints)
      print('new_pose: {0}'.format(newRightArmPose))
      self.valid = True
    else:
      rospy.loginfo('Invalid Right Arm Pose')
      print newRightArmPose
      self.valid = False
      return self
    rospy.sleep(.01)
    if newLeftJoints is not None:
      self.leftArm.go_to_joints(newLeftJoints)
    else:
      rospy.loginfo('Invalid Left Arm Pose')
    rospy.sleep(.01)
    return self

def main():
  state = SimpleArmState()

  '''MOVE ARMS OUT OF WAY'''
  state = state.raiseArms()
  state.rightArm.go_to_posture('side')
  state.leftArm.go_to_posture('side')
  state = state.alignGrippers()

  while not state.isGoalState():
    print 'Getting Next State...'
    state = state.successorState()

  state.rightArm.open_gripper()
  rospy.sleep(0.5)
  state = state.raiseArms()
  rospy.sleep(2)

  print 'Goal Reached'
  
  


if __name__ == '__main__':
    rospy.init_node('test_arm', anonymous=True)
    main()