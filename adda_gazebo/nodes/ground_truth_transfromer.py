#!/usr/bin/env python
# this script transforms the velocities of the ground_truth message from the map frame to the base link frame
# output are the velocities relative to the base_link

import rospy
import tf
import geometry_msgs
import std_msgs.msg
from nav_msgs.msg import Odometry
import roslib; roslib.load_manifest('tf2_geometry_msgs')

class GroundTruthTransformer(object):
  
  def __init__(self, namespace='ground_truth_transfromer'):
    rospy.init_node("ground_truth_transformer", anonymous = True)
    self.pub = rospy.Publisher("/ground_truth/transformed", Odometry, queue_size = 1)
    self.sub = rospy.Subscriber("/ground_truth/raw", Odometry, self.handle_pose)
    self.tl = tf.TransformListener()
    rospy.loginfo("Ground_Truth_Transformer initialized")


  def handle_pose(self, msg):
    head = msg.header
    msg.header = head
    msg.header.frame_id='map'
    msg.child_frame_id='base_footprint'

    in_lin = msg.twist.twist.linear
    in_rot = msg.twist.twist.angular
    try:
      res_vel, res_rot = self.transform_twist(in_rot, in_lin, head)
      msg.twist.twist.linear = res_vel.vector
      msg.twist.twist.angular = res_rot.vector
    except tf.LookupException:
      rospy.logwarn("No transfrom from frame 'map' to frame 'base_link'!")
    h = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()
    self.pub.publish(msg)

  #twist_rot is the velocity in euler angles (rate of roll, rate of pitch, rate of yaw; angular.x, angular.y, angular.z)
  #twist_vel is the velocity in cartesian coordinates (linear.x, linear.y, linear.z)
  def transform_twist(self, twist_rot, twist_vel, head):
    target_frame = "/base_link"
    v = geometry_msgs.msg.Vector3Stamped()
    v.header = head
    v.header.stamp = rospy.Time()
    v.vector = twist_vel
    out_vel = self.tl.transformVector3(target_frame, v)
    v.vector = twist_rot
    out_rot = self.tl.transformVector3(target_frame, v)

    return out_vel, out_rot
    #out_vel is the linear velocity w.r.t to the base_footpring, linear.x, linear.y, linear.z
    #out_Rot is the angular velocity w.r.t. to the base_footprint, angular.x, angular.y, angular.z

if __name__ == "__main__":
  ground_truth_transformer = GroundTruthTransformer()
  rospy.spin()
