#!/usr/bin/env python
import rospy
import math
from naoqi import ALProxy

class InitPose:
	def __init__(self):
		ip = rospy.get_param('~ip', '10.104.16.141')
		port = int(rospy.get_param('~port', '9559'))

		self.al = ALProxy("ALAutonomousLife", ip, port)
		self.postureProxy = ALProxy("ALRobotPosture", ip, port)
		self.motionProxy = ALProxy("ALMotion", ip, port)
		self.al.setState("disabled")
		self.postureProxy.goToPosture("StandInit", 0.5)
		for part in ["Head", "LArm", "RArm"]:
			self.motionProxy.setStiffnesses(part, 1.0)
		self.init_kinnect_pose()
	
	def init_kinnect_pose(self):
		self.motionProxy.setAngles(['LShoulderRoll', 'RShoulderRoll'], self.to_rad([57.9, -57.9]), 1.0);
		self.motionProxy.setAngles(['LShoulderPitch', 'RShoulderPitch'], self.to_rad([-94.1, -94.1]), 1.0);
		self.motionProxy.setAngles(['LElbowYaw', 'RElbowYaw'], self.to_rad([-11.5, 11.5]), 1.0);
		self.motionProxy.setAngles(['LElbowRoll', 'RElbowRoll'], self.to_rad([-61.3, 61.3]), 1.0);
		self.motionProxy.setAngles(['LWristYaw', 'RWristYaw'], self.to_rad([7.0, -7.0]), 1.0);
		self.motionProxy.setAngles(['LHand', 'RHand'], self.to_rad([0.99, 0.99]), 1.0);

	def to_rad(self, angles):
		return map(lambda x: float(x)*math.pi/180.0, angles)

if __name__ == '__main__':
	InitPose()
