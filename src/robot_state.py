#! /usr/bin/env python
import roslib; roslib.load_manifest('RobotController')
import rospy
import tf
#import PyKDL
#import control_primitives
#import contact_reflex
from RobotController.msg import RobotState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float32

class orientation(object):
	def __init__(self):
		self._quaternion = [0,0,0,1]
		self._RPY = [0,0,0]
	def SetQuaternion(self,quaternion):
		self._quaternion = quaternion
		self._RPY = tf.transformations.euler_from_quaternion(quaternion)
	def GetRPY(self):
		return self._RPY

class cb_joints(object):
	def __init__(self,args):
		self.cbJointNames = args
		self._jnt = [0.0 for k in self.cbJointNames]

	def SetAll(self,jnt):
		self._jnt = jnt

	def Get(self,name):
		return self._jnt[self.cbJointNames.index(name)]

	def GetAll(self):
		return self._jnt

class robot_state(object):
	def __init__(self,robot_joints):
		self._time = rospy.Time(0)
		self._JointPos = cb_joints(robot_joints)
		self._JointVel = cb_joints(robot_joints)
		self._JointEff = cb_joints(robot_joints)
		self._orientation = orientation()

	def UpdateState(self,StateMessage):
		self._time = StateMessage.header.stamp
		self._JointPos.SetAll(StateMessage.position)
		self._JointVel.SetAll(StateMessage.velocity)
		self._JointEff.SetAll(StateMessage.effort)
		quaternion = [StateMessage.orientation.x, StateMessage.orientation.y, StateMessage.orientation.z, StateMessage.orientation.w]
		self._orientation.SetQuaternion(quaternion)
		#self._angular_vel.x(self._vel_x_filter.update(StateMessage.angular_velocity.x))
		#self._angular_vel.y(self._vel_y_filter.update(StateMessage.angular_velocity.y))
		#self._angular_vel.z(self._vel_z_filter.update(StateMessage.angular_velocity.z))
		#self._l_foot_force = self._l_contact_filter.update(StateMessage.l_foot.force.z)
		#self._r_foot_force = self._r_contact_filter.update(StateMessage.r_foot.force.z)
		#self._r_reflex['GC'],self._l_reflex['GC'] = self._GC.update(self._r_foot_force,self._l_foot_force)
		pass
	def GetJointPos(self,joint='None'):
		if joint == 'None':
		   return self._JointPos.GetAll()
		return self._JointPos.Get(joint)
		
	#def GetContactForce(self,leg):
	#    if leg == 'l':
	#        return self._l_foot_force
	#    if leg == 'r':
	#        return self._r_foot_force
	#    return ValueError
	def GetJointVel(self,joint):
		return self._JointVel.Get(joint)
	def GetJointEff(self,joint):
		return self._JointEff.Get(joint)
	def GetAngVel(self):
		return self._angular_vel
	#def GetEvents(self,leg):
	#    if leg == 'l':
	#        return self._l_reflex
	#    if leg == 'r':
	#        return self._r_reflex
	#    return ValueError
############################################################################################################################################
#                                                            TEST                                                                          #
############################################################################################################################################
if __name__ == '__main__':
	class test(object):
		def __init__(self):
			self.RS = robot_state()
			self.sub = rospy.Subscriber('/atlas/atlas_state',AtlasState,self._cb)
			self.AngVelPub = rospy.Publisher('/RS_test/ang_vel',Vector3)
			self.l_GCPub = rospy.Publisher('/RS_test/l_GC',Float32)
			self.r_GCPub = rospy.Publisher('/RS_test/r_GC',Float32)     
		def _cb(self,msg):
			self.RS.UpdateState(msg)
			ang_vel = self.RS.GetAngVel()
			l_rflx = self.RS.GetEvents('l')
			r_rflx = self.RS.GetEvents('r')
			self.AngVelPub.publish(Vector3(ang_vel.x(),ang_vel.y(),ang_vel.z()))
			self.l_GCPub.publish(Float32(l_rflx['GC']))
			self.r_GCPub.publish(Float32(r_rflx['GC']))
	rospy.init_node('TestAtlasState')
	T = test()
	rospy.spin() 



