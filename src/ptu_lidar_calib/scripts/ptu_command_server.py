#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from ptu_lidar_calib.srv import GoTo, GoToResponse
from sensor_msgs.msg import JointState

import math
import time
from threading import Lock

class PTUCommander:
    def __init__(self):
        self.cmd_topic   = rospy.get_param("~command_topic", "/joint_command")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")
        # Joint name order MUST match your Arduino expectation: [tilt, pan]
        self.tilt_name   = rospy.get_param("~tilt_joint", "joint1")
        self.pan_name    = rospy.get_param("~pan_joint",  "joint2")
        self.rate_hz     = rospy.get_param("~publish_rate", 50.0)

        # Limits (deg) for safety/clamping
        self.pan_limit_deg  = rospy.get_param("~pan_limit_deg", 60.0)
        self.tilt_limit_deg = rospy.get_param("~tilt_limit_deg", 45.0)

        self.pub = rospy.Publisher(self.cmd_topic, JointState, queue_size=10)
        self.js_lock = Lock()
        self.last_js = None
        rospy.Subscriber(self.joint_state_topic, JointState, self._js_cb)

        # Current target (radians)
        self.tgt_tilt = 0.0
        self.tgt_pan  = 0.0
        self.hold_active = True

        rospy.Service("~goto", GoTo, self._srv_goto)

        rospy.loginfo("PTU command server ready at %s", rospy.get_name())
        self._publisher_loop()

    def _js_cb(self, msg):
        with self.js_lock:
            self.last_js = msg

    def _closest_positions(self):
        """Return (tilt, pan) from /joint_states in radians, or (None, None)."""
        with self.js_lock:
            js = self.last_js
        if js is None:
            return (None, None)

        tilt = None
        pan = None
        for i, name in enumerate(js.name):
            if name == self.tilt_name:
                tilt = js.position[i]
            elif name == self.pan_name:
                pan = js.position[i]
        return (tilt, pan)

    def _srv_goto(self, req):
        # Clamp and convert
        pan_deg  = max(-self.pan_limit_deg,  min(self.pan_limit_deg,  req.pan_deg))
        tilt_deg = max(-self.tilt_limit_deg, min(self.tilt_limit_deg, req.tilt_deg))
        pan_rad  = math.radians(pan_deg)
        tilt_rad = math.radians(tilt_deg)
        tol_rad  = math.radians(max(0.01, req.pos_tolerance_deg if req.pos_tolerance_deg > 0.0 else 0.5))
        timeout  = req.timeout_s if req.timeout_s > 0.0 else 10.0

        self.tgt_pan  = pan_rad
        self.tgt_tilt = tilt_rad

        t0 = time.time()
        if req.wait:
            # Wait until within tolerance or timeout
            while not rospy.is_shutdown():
                cur_tilt, cur_pan = self._closest_positions()
                if cur_tilt is not None and cur_pan is not None:
                    if abs(cur_tilt - self.tgt_tilt) <= tol_rad and abs(cur_pan - self.tgt_pan) <= tol_rad:
                        return GoToResponse(True, "Reached: pan=%.2f°, tilt=%.2f°" % (pan_deg, tilt_deg))
                if (time.time() - t0) > timeout:
                    return GoToResponse(False, "Timeout before reaching target (maybe lower tolerance or increase timeout).")
                rospy.sleep(0.02)

        return GoToResponse(True, "Commanded: pan=%.2f°, tilt=%.2f°" % (pan_deg, tilt_deg))

    def _publisher_loop(self):
        r = rospy.Rate(self.rate_hz)
        msg = JointState()
        msg.name = [self.tilt_name, self.pan_name]  # [tilt, pan] order to match Arduino
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            msg.position = [self.tgt_tilt, self.tgt_pan]
            # velocities/effort left empty
            self.pub.publish(msg)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("ptu_command_server")
    PTUCommander()
