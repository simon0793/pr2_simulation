import arm

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

class SimpleArmState:
  ''' Class for representing just the state of PR2 arms (no additional objects)
  '''
  def __init__(self):
    '''
    '''
    self.rightArm = arm.Arm('right')
    self.leftArm = arm.Arm('left')
    self.rightArm.sim.env.SetViewer('qtcoin')
    # self.leftArm.sim.env.SetViewer('qtcoin')

  def isGoalState(self):
    ''' Returns true when goal state is reached
      by default always returns true
    '''

    ''' ------ CHANGE CODE BELOW THIS POINT ------ 
    Modify target postions or define your own goal state
    '''
    rightArmPose = self.rightArm.get_pose()
    targetX, targetY, targetZ = [0.630, -0.740,  0.775] # Assign a target position
    currentX, currentY, currentZ = rightArmPose.position.array
    acceptableAbsError = 0.1
    rightValid = abs(targetX - currentX) <= acceptableAbsError and abs(targetY - currentY) <= acceptableAbsError and abs(targetZ - currentZ) <= acceptableAbsError # Modify to check the current state
    leftArmPose = self.leftArm.get_pose()
    targetX, targetY, targetZ = [0.630, 0.177,  0.775] # Assign a target position
    currentX, currentY, currentZ = leftArmPose.position.array
    acceptableAbsError = 0.1
    leftValid = abs(targetX - currentX) <= acceptableAbsError and abs(targetY - currentY) <= acceptableAbsError and abs(targetZ - currentZ) <= acceptableAbsError # Modify to check the current state
    return rightValid and leftValid

    ''' ------ CHANGE CODE ABOVE THIS POINT ------ '''

  def succesorState(self):
    ''' Updates current state to the next succesor state
      by default it just returns current state
    '''
    rightArmPose = self.rightArm.get_pose()
    leftArmPose = self.leftArm.get_pose()
    newRightArmPose = tfx.pose(rightArmPose) # Create new pose
    newLeftArmPose = tfx.pose(leftArmPose) # Create new pose

    ''' ------ CHANGE CODE BELOW THIS POINT ------ '''

    newRightArmPose.position = rightArmPose.position.array # either update by adding to xyz offset array of the form  "+ [#, #, #]" or assign new position, defualt keeps position the same
    newLeftArmPose.position = leftArmPose.position.array

    ''' ------ CHANGE CODE ABOVE THIS POINT ------ '''


    newRightJoints = self.rightArm.ik(newRightArmPose) # Updates arm pose if valid
    newLeftJoints = self.leftArm.ik(newLeftArmPose)
    if newRightJoints is not None:
      self.rightArm.go_to_joints(newRightJoints)
      print('new_pose: {0}'.format(newRightArmPose))
    else:
      rospy.loginfo('Invalid Right Arm Pose')
    rospy.sleep(.01)
    if newLeftJoints is not None:
      self.leftArm.go_to_joints(newLeftJoints)
    else:
      rospy.loginfo('Invalid Left Arm Pose')
    rospy.sleep(.01)
    return self # Modify to generate a valid succesor states, does not update state if pose is invalid and instead reverts to last valid state


def main():
  state = SimpleArmState()
  while not state.isGoalState():
    print 'Getting Next State...'
    state = state.succesorState()
  print 'Goal Reached'
  
  


if __name__ == '__main__':
    rospy.init_node('test_arm', anonymous=True)
    main()