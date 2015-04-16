#! /usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from my_kinnect.msg import SkeletonCoords

class Tracker():

    def __init__(self):
        rospy.init_node('tracker_mykinect', anonymous=True)
	self.myFrame = 'kinect_link'
	self.pub_trans = rospy.Publisher(self.myFrame, SkeletonCoords, queue_size=10)
        rate = rospy.get_param('~rate', 3)

        r = rospy.Rate(rate)


        self.tf = tf.TransformListener()
        rospy.loginfo("Start tracking for 5s...")
        rospy.sleep(5.0)
        rospy.loginfo("Tracking started!")

	for i in range(1, 4):	
            try:
                 self.target_joint = "{0}_{1}".format(rospy.get_param('~target_joint', '/head'), i)
                 break
            except:
                 pass

        while not rospy.is_shutdown():
	    trans, rot = self.tf.lookupTransform('/openni_depth_frame', self.target_joint, rospy.Duration())
            vec = Vector3(*trans)
            rt = Quaternion(*rot)
            h = Header(stamp=rospy.Time.now())
            head = SkeletonCoords(trans=vec, rot=rt, header=h)
            self.pub_trans.publish(head)
            r.sleep()

if __name__ == '__main__':
    try:
        Tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
