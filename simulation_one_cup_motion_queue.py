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
TARGET_GREEN = [0.6, -0.405, 0.80]
ALIGN_GRASP_GREEN = [0.7, -0.405, 0.80]
RAISED_GREEN = [0.7, -0.405, 1.0]
RAISED_DEST_GREEN = [0.7, 0.0, 1.0]
DEST_GREEN = [0.7, 0.0, 0.81]
DEST_GREEN_REMOVED = [0.45, 0.0, 1.0]

'''OUTPUT FILE TO WRITE TO'''
FILENAME = '/home/simon/output.txt'
FILE = None


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
    rightArmPose = self.rightArm.get_pose()
    self.currentGoal = rightArmPose.position.array # Set goal as default to current starting position

  def setCurrentGoalPosition(self, newGoalPosition):
    self.currentGoal = newGoalPosition


  def isGoalState(self, acceptableAbsError = 0.1):
    ''' Returns true when goal state is reached
      by default always returns true
    '''

    ''' ------ CHANGE CODE BELOW THIS POINT ------ '''
    rightArmPose = self.rightArm.get_pose()
    targetX, targetY, targetZ = self.currentGoal
    currentX, currentY, currentZ = rightArmPose.position.array
    return (abs(targetX - currentX) <= acceptableAbsError and abs(targetY - currentY) <= acceptableAbsError and abs(targetZ - currentZ) <= acceptableAbsError) # Modify to check the current state

    ''' ------ CHANGE CODE ABOVE THIS POINT ------ '''


  def successorState(self):
    ''' Updates current state to the next successor state
      by default it just returns current state
    '''
    if self.valid:
      newPosition = self.currentGoal
    else:
      print "The last successor state was invalid"
      print "Enter a custom pose x y z"
      coord = raw_input().split()
      newPosition = [float(x) for x in coord]
    newRightArmPose = tfx.pose(self.rightArm.get_pose()) # only want to change position not angles
    newRightArmPose.position = newPosition
    newRightJointsList = self.graspPlanner.get_grasp_joint_trajectory(self.rightArm.get_joints(), newRightArmPose.position, n_steps=10)
    print newRightJointsList
    if newRightJointsList is None:
      rospy.loginfo('Invalid Right Arm Pose')
      print newPosition
      self.valid = False
    else:
      for newRightJoints in newRightJointsList:
        writeToOutput(newRightJoints)
      self.rightArm.execute_joint_trajectory(newRightJointsList)
      self.valid = True
    rospy.sleep(.01)
    return self # Modify to generate a valid successor states, does not update state if pose is invalid and instead reverts to last valid state

  def grasp(self):
    self.rightArm.close_gripper()
    print "Closing Gripper..."
    rospy.sleep(2)

  def release(self):
    self.rightArm.open_gripper()
    print "Opening Gripper..."
    rospy.sleep(10)


  def clear_arms(self):
    self.raiseArms()
    self.rightArm.go_to_posture('side')
    self.leftArm.go_to_posture('side')
    self.alignGrippers()

  def alignGrippers(self):
    rightArmPose = self.rightArm.get_pose()
    leftArmPose = self.leftArm.get_pose()
    newLeftArmPose = tfx.pose(leftArmPose.position, tfx.tb_angles(0.0, 0.0, 0.0)) # Create new pose

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

class queuePlanner():
  ''' Class for planning motion using a simple queue of positions
  '''
  def __init__(self, positionQueue = []):
    self.state = SimpleArmState()
    self.goalPositionQueue = positionQueue
    self.index = 0

  def _pop(self):
    if self.index < len(self.goalPositionQueue):
      self.index += 1
      return self.goalPositionQueue[self.index - 1]
    else:
      return None

  def run(self):
    currentGoal = self._pop();
    while currentGoal is not None:
      if isinstance(currentGoal, str):
        if currentGoal == "CLEAR":
          self.state.clear_arms()
        elif currentGoal == "GRASP":
          self.state.grasp()
        elif currentGoal == "RELEASE":
          self.state.release()
        else:
          print "ERROR INVALID COMMAND EXITING"
          return -1
      else:
        self.state.setCurrentGoalPosition(currentGoal)
        while not self.state.isGoalState():
          print 'Getting Next State...'
          self.state = self.state.successorState()
      currentGoal = self._pop()
    print 'Goal Reached'
    return 0

  # def step(self):

  # def next(self):

  # def skipStep(self):

  # def skipNext(self):

  # def setBreak(self):


def main():
  global FILE
  FILE = open(FILENAME, 'w')

  goalPositions = ["CLEAR", TARGET_GREEN, ALIGN_GRASP_GREEN, "GRASP", RAISED_GREEN, RAISED_DEST_GREEN, DEST_GREEN, "RELEASE", DEST_GREEN_REMOVED, "CLEAR"]
  qPlan = queuePlanner(goalPositions)
  qPlan.run()

  FILE.close()


def writeToOutput(data):
  if FILE is not None:
    FILE.write(str(data) + '\n')
  
  


if __name__ == '__main__':
    rospy.init_node('test_arm', anonymous=True)
    main()