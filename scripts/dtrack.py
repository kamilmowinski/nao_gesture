#! /usr/bin/env python
import rospy
import tf
import math
from naoqi import ALProxy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from my_kinnect.msg import SkeletonCoords

class Tracker():

    def __init__(self):
	rospy.init_node('tracker_mykinect', anonymous=True)
	self.myFrame = 'kinect_link'
	self.r = rospy.Rate(20)		#rospy.Rate(1) = 1Hz

	ip = rospy.get_param('~ip', '10.104.16.50')
	port = int(rospy.get_param('~port', '9559'))
	#self.al = ALProxy("ALAutonomousLife", ip, port)
	#self.postureProxy = ALProxy("ALRobotPosture", ip, port)
	#self.motionProxy = ALProxy("ALMotion", ip, port)
	#self.al.setState("disabled")
	#self.postureProxy.goToPosture("StandInit", 0.5)
	#for part in ["Head", "LArm", "RArm"]:
	#	self.motionProxy.setStiffnesses(part, 1.0)
	#	self.init_kinnect_pose()
	self.tf = tf.TransformListener()
	rospy.loginfo("Start tracking for 3s...")
        rospy.sleep(3.0)
	rospy.loginfo("Tracking started!")
	self.hoh = 0.86 #wysokosc na jakiej znajduje sie kinect
	for i in range(1, 4):	
		try:
			#self.target_joint = "{0}_{1}".format(rospy.get_param('~target_joint', '/head'), i)	#uzysukuje nazwe zalezna od szkieletu
			self.j_head = "{0}_{1}".format(rospy.get_param('~target_joint', '/head'), i)
			self.j_l_hand = "{0}_{1}".format(rospy.get_param('~target_joint', '/left_hand'), i) #widzi odbicie lustrzane prawa to lewa
			self.j_r_hand = "{0}_{1}".format(rospy.get_param('~target_joint', '/right_hand'), i)
			self.j_l_shoul = "{0}_{1}".format(rospy.get_param('~target_joint', '/left_shoulder'), i)
			self.j_torso = "{0}_{1}".format(rospy.get_param('~target_joint', '/torso'), i)
			self.j_l_ehand = "{0}_{1}".format(rospy.get_param('~target_joint', '/left_elbow'), i)
			self.j_r_ehand = "{0}_{1}".format(rospy.get_param('~target_joint', '/right_elbow'), i)
			break
		except:
			pass

	ip = rospy.get_param('~ip', '10.104.16.50')
	port = int(rospy.get_param('~port', '9559'))
	self.al = ALProxy("ALAutonomousLife", ip, port)
	self.postureProxy = ALProxy("ALRobotPosture", ip, port)
	self.tts = ALProxy("ALTextToSpeech", ip, port)
	self.motionProxy = ALProxy("ALMotion", ip, port)
	self.al.setState("disabled")
	self.postureProxy.goToPosture("StandInit", 0.5)
	for part in ["Head", "LArm", "RArm"]:
		self.motionProxy.setStiffnesses(part, 1.0)
	#self.init_kinnect_pose()

    def init_kinnect_pose(self):
	self.motionProxy.setAngles(['LShoulderRoll', 'RShoulderRoll'], self.to_rad([57.9, -57.9]), 1.0);
	self.motionProxy.setAngles(['LShoulderPitch', 'RShoulderPitch'], self.to_rad([-94.1, -94.1]), 1.0);
	self.motionProxy.setAngles(['LElbowYaw', 'RElbowYaw'], self.to_rad([-11.5, 11.5]), 1.0);
	self.motionProxy.setAngles(['LElbowRoll', 'RElbowRoll'], self.to_rad([-61.3, 61.3]), 1.0);
	self.motionProxy.setAngles(['LWristYaw', 'RWristYaw'], self.to_rad([7.0, -7.0]), 1.0);
	self.motionProxy.setAngles(['LHand', 'RHand'], self.to_rad([0.99, 0.99]), 1.0);

    def to_rad(self, angles):
	return map(lambda x: float(x)*math.pi/180.0, angles)

	
    def init_kinnect_pose(self):
	self.motionProxy.setAngles(['LShoulderRoll', 'RShoulderRoll'], self.to_rad([57.9, -57.9]), 1.0);
	self.motionProxy.setAngles(['LShoulderPitch', 'RShoulderPitch'], self.to_rad([-94.1, -94.1]), 1.0);
	self.motionProxy.setAngles(['LElbowYaw', 'RElbowYaw'], self.to_rad([-11.5, 11.5]), 1.0);
	self.motionProxy.setAngles(['LElbowRoll', 'RElbowRoll'], self.to_rad([-61.3, 61.3]), 1.0);
	self.motionProxy.setAngles(['LWristYaw', 'RWristYaw'], self.to_rad([7.0, -7.0]), 1.0);
	self.motionProxy.setAngles(['LHand', 'RHand'], self.to_rad([0.99, 0.99]), 1.0);

    def to_rad(self, angles):
	return map(lambda x: float(x)*math.pi/180.0, angles)

    def nao_hello(self):
	self.postureProxy.goToPosture("Standing",1)
	self.motionProxy.setAngles('RShoulderPitch',self.to_rad([-60.0])[0],1.0)
	self.motionProxy.setAngles('RShoulderRoll',self.to_rad([-1.6])[0],1.0)
	self.motionProxy.setAngles('RElbowRoll',self.to_rad([37.5])[0],1.0)
	self.motionProxy.setAngles('RElbowYaw',self.to_rad([79.1])[0],1.0)
	self.motionProxy.setAngles('RWristYaw',self.to_rad([57.0])[0],1.0)
	self.motionProxy.openHand('RHand')
	self.tts.say("Hello my friend")
	for i in range(1,2):
		self.motionProxy.setAngles('RShoulderPitch',self.to_rad([-25.0])[0],1.0)
		self.motionProxy.setAngles('RShoulderPitch',self.to_rad([-60.0])[0],1.0)
    


    def hello_process(self):
	self.count = 0
	self.side = 0		#left 0 right 1
	while not rospy.is_shutdown():
		trans_head, rot= self.tf.lookupTransform('/openni_depth_frame', self.j_head, rospy.Duration())
		vec_head = Vector3(*trans_head)
		trans_hand, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_hand, rospy.Duration())
		vec_hand = Vector3(*trans_hand)
		trans_shoul, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_shoul, rospy.Duration())
		vec_shoul = Vector3(*trans_shoul)
		trans_torso, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_torso, rospy.Duration())
		vec_torso = Vector3(*trans_torso)
		if (vec_hand.z > vec_torso.z):
			#print('reka w gorze')
			#print(vec_shoul.y-vec_hand.y)
			if (not self.side):
				if (vec_shoul.y-vec_hand.y > 0.1):
					self.count+=1
					self.side = 1
			else:
				if (vec_shoul.y-vec_hand.y < (-0.2) ):
					self.count+=1
					self.side = 0
		if (self.count >= 4):
			print('HELLO')
			self.nao_hello()
			self.count = 0
		self.r.sleep()		#sleep rospy.Rate

    def sensei_process(self):
	while not rospy.is_shutdown():
		trans_head, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_head, rospy.Duration())
		trans_rh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_r_hand, rospy.Duration())
		trans_lh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_hand, rospy.Duration())
		trans_tor, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_torso, rospy.Duration())
		vec_head = Vector3(*trans_head)
		vec_rh = Vector3(*trans_rh)
		vec_lh = Vector3(*trans_lh)
		vec_tor = Vector3(*trans_tor)
		#print(str(vec_head.x) + "\t\t" + str(vec_head.y) + "\t\t" + str(vec_head.z))
		#print(str(vec_rh.x) + "\t\t" + str(vec_rh.y) + "\t\t" + str(vec_rh.z))
		#print(str(vec_lh.x) + "\t\t" + str(vec_lh.y) + "\t\t" + str(vec_lh.z))
		#print(str(vec_tor.x) + "\t\t" + str(vec_tor.y) + "\t\t" + str(vec_tor.z))
		#print('')
		#print (str(vec_rh.x-vec_tor.x) + "\t\t" + str(vec_lh.x-vec_tor.x) + "\t\t" + str(vec_rh.z-vec_tor.z) + "\t\t" + str(vec_lh.z-vec_tor.z))	
		ukl = False
		if ( (vec_rh.x > (vec_tor.x - 0.15) and vec_rh.x < vec_tor.x) \
			 and  (vec_lh.x > (vec_tor.x - 0.15) and vec_lh.x < vec_tor.x) \
			 and  (vec_rh.z > (vec_tor.z - 0.2) and vec_rh.z < (vec_tor.z + 0.2)) \
			 and  (vec_lh.z > (vec_tor.z - 0.2) and vec_lh.z < (vec_tor.z+ 0.2))):
			print("zlozone rece")
			ukl = True;
		if (ukl):
			starthead = vec_head.x
			count = 0
			while (ukl):
				trans_head, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_head, rospy.Duration())
				self.r.sleep()
				vec_head = Vector3(*trans_head)
				currhead = vec_head.x 
				if (count == 0 and starthead-currhead > 0.1):
					print("0")
					count +=1
					continue
				if (count == 1 and starthead-currhead > 0.175):
					print("1")
					count +=1
					continue
				if (count == 2 and starthead-currhead > 0.25):
					print("2")
					count +=1
					continue
				if (count == 3 and starthead-currhead < 0.175):
					print("3")
					count +=1
					continue
				if (count == 4 and starthead-currhead < 0.1):
					print("uklon!")
					ukl = False
		self.r.sleep()
		#print("out!")

    def goto_process(self):
	while not rospy.is_shutdown():
		trans_lh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_hand, rospy.Duration())
		trans_leh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_ehand, rospy.Duration())
		B = Vector3(*trans_lh)
		A = Vector3(*trans_leh)
		#print(str(A.z) + "\t\t" + str(B.z))
		if (A.z < B.z):
			self.r.sleep()
			continue
		# dlugosc ramienia
		arm_length = math.sqrt( math.pow(A.x-B.x,2) + math.pow(A.y-B.y,2) + math.pow(A.z-B.z,2))
		#print(arm_length)

		# kat miedzy ramieniem a postora prostopadla do podloza
		alfa = math.acos((A.z-B.z)/arm_length)
		#print(alfa)

		# dlugosc odcinka do ziemi od konca reki pod katem alfa
		lenght_to_ground = (B.z + self.hoh)/math.cos(alfa)
		#print(lenght_to_ground)	

		# kat medzy ramieniem a kinectem 
		beta = math.asin((B.y-A.y)/arm_length)
		#print(beta)

		# przesuniecie x  miedzy reka a punktem na podlodze
		dx = math.sin(alfa)*lenght_to_ground
		#print(dx)

		# przesuniecie y medzy reka a punktem na podlodze
		dy = math.sin(beta)*lenght_to_ground
		#print(dy)

		#punk na podlodze w stosunku do kinecta
		pkt = Vector3()
		pkt.z = 0
		pkt.x = B.x-dx
		pkt.y = B.y +dy
		print (str(pkt.x) +"\t\t" +str(pkt.y))
		self.r.sleep()

    def tome_process(self):
	count = 0
	total = 0
	oldrx = 0
	oldlx = 0
	oldelz = 0
	olderz = 0
	while not rospy.is_shutdown():
		trans_rh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_r_hand, rospy.Duration())
		trans_lh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_hand, rospy.Duration())
		trans_erh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_r_ehand, rospy.Duration())
		trans_elh, rot = self.tf.lookupTransform('/openni_depth_frame', self.j_l_ehand, rospy.Duration())
		vec_rh = Vector3(*trans_rh)
		vec_lh = Vector3(*trans_lh)
		vec_reh = Vector3(*trans_erh)
		vec_leh = Vector3(*trans_elh)
		#print(str(vec_rh.x) + "\t\t" + str(vec_rh.y) + "\t\t" + str(vec_rh.z))
		#print(str(vec_reh.x) + "\t\t" + str(vec_reh.y) + "\t\t" + str(vec_reh.z))
		#print(' ')
		#print(str(vec_lh.x) + "\t\t" + str(vec_lh.y) + "\t\t" + str(vec_lh.z))
		#print(str(vec_leh.x) + "\t\t" + str(vec_leh.y) + "\t\t" + str(vec_leh.z))
		#print(' ')
		#print(' ')
		
		#rece pod katem 90 stopni do powierzchni
		if(  (-0.05 < (vec_rh.z - vec_reh.z) < 0.05) and (-0.05 < (vec_lh.z - vec_leh.z) < 0.05) and count == 0 ):
			print("start")
			count = 1
			oldrx = vec_rh.x
			oldlx = vec_lh.x
			oldelz = vec_leh.z
			olderz = vec_reh.z
		if(  (0.15 > (vec_rh.x - oldrx) > 0.05) and (0.15 > (vec_lh.x - oldlx) > 0.05) and count == 1):
			print("sek2")
			count = 2
			oldrx = vec_rh.x
			oldlx = vec_lh.x
		if(  (vec_rh.x - oldrx) > 0.15 and (vec_lh.x - oldlx) > 0.15 and count == 2 and\
				 (-0.05 < (vec_reh.z - olderz) < 0.05) and  (-0.05 < (vec_leh.z - oldelz) < 0.05)):
			count = 0
			print("sek3")
			total += 1
		if (total == 2):
			print("Come to ME!")
			total = 0
		if ((vec_leh.z - vec_lh.z > 0.15 or (vec_reh.z - vec_rh.z > 0.15)) and count != 0):
			print("RESET!")
			total = 0
			count = 0
		self.r.sleep()

if __name__ == '__main__':
    try:
        tr = Tracker()
        task = rospy.get_param('~task', None)
        if task == 'hello':
	    tr.hello_process()
	if task == 'sensei':
            tr.sensei_process()
	if task == 'goto':
            tr.goto_process()
	if task == 'tome':
            tr.tome_process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
