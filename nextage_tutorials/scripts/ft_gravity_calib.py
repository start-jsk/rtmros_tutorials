#!/usr/bin/env python

import rospy
import tf
import numpy as np
import yaml
from geometry_msgs.msg import WrenchStamped


class FTGravityCalib:
    def __init__(self):
        self.ft_topic = rospy.get_param("~ft_topic", "/left_ft/left_ft/force_torque")
        self.sensor_frame = rospy.get_param("~sensor_frame", "LARM_JOINT5_Link")
        self.base_frame = rospy.get_param("~base_frame", "WAIST")

        self.output_yaml = rospy.get_param("~yaml_path", "ft_gravity.yaml")

        self.listener = tf.TransformListener()

        self.ft_data = []
        self.gravity = np.array([0, 0, -9.81])

        rospy.Subscriber(self.ft_topic, WrenchStamped, self.cb_ft)
        rospy.on_shutdown(self.solve)

        rospy.loginfo("=== FT Gravity Calibration Started ===")
        rospy.loginfo("Collecting FT data...  (Ctrl+C to finish)")
        rospy.loginfo("Will save to: %s" % self.output_yaml)

    def cb_ft(self, msg):
        """Record FT and gravity direction vector."""
        F = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.sensor_frame,     # target
                self.base_frame,       # source
                rospy.Time(0)
            )
        except (tf.LookupException, tf.ExtrapolationException):
            return

        R = tf.transformations.quaternion_matrix(rot)[:3, :3]

        g_sensor = R.dot(self.gravity)

        self.ft_data.append((F, g_sensor))

        if len(self.ft_data) % 50 == 0:
            rospy.loginfo("Collected samples: %d" % len(self.ft_data))

    def solve(self):
        if len(self.ft_data) == 0:
            rospy.logwarn("No data collected. YAML not saved.")
            return

        rospy.loginfo("Solving least squares...")

        # F = m*g + bias
        A = []
        b = []

        for F, g in self.ft_data:
            A.append([g[0], 1, 0, 0])
            A.append([g[1], 0, 1, 0])
            A.append([g[2], 0, 0, 1])
            b.extend(F)

        A = np.array(A, dtype=float)
        b = np.array(b, dtype=float)

        x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        mass = float(x[0])
        bias = x[1:4].tolist()

        rospy.loginfo("Estimated mass: %.4f kg" % mass)
        rospy.loginfo("Estimated bias: %s" % bias)

        calib = {"mass": mass, "bias": bias}

        try:
            import os
            os.makedirs(os.path.dirname(self.output_yaml), exist_ok=True)
        except:
            pass
        with open(self.output_yaml, "w") as f:
            yaml.safe_dump(calib, f, default_flow_style=False)
        rospy.loginfo("Saved calibration → %s" % self.output_yaml)


if __name__ == "__main__":
    rospy.init_node("ft_gravity_calib")
    FTGravityCalib()
    rospy.spin()
