#! /usr/bin/env python
import actionlib
import rospy
import std_msgs
import intersection.msg #Depends on package name
from std_msgs.msg import Int32

class IntersectionAction(object):
	_goal = intersection.msg.IntersectionGoal()
	_result = intersection.msg.IntersectionResult()
	_feedback = intersection.msg.IntersectionFeedback()
	def __init__(self):
		self.pub_lane = rospy.Publisher('intersection/lane', Int32, queue_size=10)
		self.pub_offset = rospy.Publisher('intersection/offset', Int32, queue_size=10)
		self.action_name = "intersection"
		self._as = actionlib.SimpleActionServer(self._action_name, intersection.msg.InterectionAction, exectue_cb = self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self,goal):
		r = rospy.Rate(1) #Tunable Parameter
		done = False
		emergency = False
		while(not done):
			if self._as.is_preempt_requested():
				self._result.success = False
				self._result.isEmergency = True
				#Implement current lane
				self._as.set_preempted(self._result)
				return
			if(goal.direction == -1):
				self.pub_lane.publish(goal.direction)
				self.pub_offset.publish(250)
				pass

			elif(goal.direction == 0):
				#Straight road implementation
				pass

			elif(goal.direction == 1):
				self.pub_lane.publish(goal.direction)
				self.pub_offset.publish(-150)
				pass

			r.sleep()

		if(not emergency):
			self._result.success = True
			self._result.isEmergency = False
			#Implement current lane
			self._as.set_succeeded(self._result)
			return
		
		else:
			self._result.success = False
			self._result.isEmergency = True
			#Implement current lane
			self._as.set_succeeded(self._result)
			return

if __name__=="__main__":
    try:
        rospy.init_node('intersection')
        server = IntersectionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Interupted")