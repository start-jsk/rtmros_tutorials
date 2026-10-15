#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import WrenchStamped


class FTGravityCompensate:
    def __init__(self):
        self.ft_topic = rospy.get_param("~input_topic")
        self.out_topic = rospy.get_param("~output_topic")
        self.sensor_frame = rospy.get_param("~sensor_frame")
        self.base_frame = rospy.get_param("~base_frame")

        self.mass = rospy.get_param("~mass")
        self.bias = np.array(rospy.get_param("~bias"))  # [bx, by, bz]

        self.gravity = np.array([0, 0, -9.81])

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher(self.out_topic, WrenchStamped, queue_size=10)

        rospy.Subscriber(self.ft_topic, WrenchStamped, self.cb)

        rospy.loginfo("=== FT Gravity Compensation Started ===")
        rospy.loginfo("input_topic:  %s" % self.ft_topic)
        rospy.loginfo("output_topic: %s" % self.out_topic)
        rospy.loginfo("mass: %.4f" % self.mass)
        rospy.loginfo("bias: %s" % self.bias.tolist())

    def cb(self, msg):
        F = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

        try:
            (_, rot) = self.listener.lookupTransform(
                self.sensor_frame,  # target
                self.base_frame,    # source
                rospy.Time(0)
            )
        except (tf.LookupException, tf.ExtrapolationException):
            return

        R = tf.transformations.quaternion_matrix(rot)[:3, :3]

        g_sensor = R.dot(self.gravity)

        Fg = self.mass * g_sensor

        Fcorr = F - Fg - self.bias

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.x = Fcorr[0]
        out.wrench.force.y = Fcorr[1]
        out.wrench.force.z = Fcorr[2]

        out.wrench.torque.x = msg.wrench.torque.x
        out.wrench.torque.y = msg.wrench.torque.y
        out.wrench.torque.z = msg.wrench.torque.z

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("ft_gravity_compensate")
    node = FTGravityCompensate()
    rospy.spin()
