#! /usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from atemr_msgs.srv import RobotAwareness, RobotAwarenessResponse

class SpeedRamp:
    def __init__(self, tmux_vel_topic="tmux/cmd_vel", reactive_vel_topic="reactive/cmd_vel", out_vel_topic="base_controller/cmd_vel"):
        self.last_twist_send_time = rospy.Time.now()
        self.awareness_srvr = rospy.Service('RobotAwareness', RobotAwareness, self.awareness_cb)
        self.output_vel_pub = rospy.Publisher(out_vel_topic, Twist, queue_size=1)

        self.use_reactive_input = False
        self.tmux_vel_topic = tmux_vel_topic
        self.reactive_vel_topic = reactive_vel_topic
        self.tmux_sub = rospy.Subscriber(self.tmux_vel_topic, Twist, self.speed_cb)

        self.target_twist = Twist()
        self.last_twist = Twist()
        self.vel_scales = [0.1] * 2
        self.vel_scales[0] = self.fetch_param('~angular_scale', 0.1) # default to very slow
        self.vel_scales[1] = self.fetch_param('~linear_scale', 0.1)
        self.vel_ramps = [1.0] * 2
        self.vel_ramps[0] = self.fetch_param('~angular_accel', 1.0) # units: meter per second^2
        self.vel_ramps[1] = self.fetch_param('~linear_accel', 1.0)
    
    def awareness_cb(self, req):
        if(not self.use_reactive_input and req.set_awareness.data): #if switching from RAW manual to REACTIVE manual control
            self.tmux_sub.unregister()
            self.reactive_sub = rospy.Subscriber(self.reactive_vel_topic, Twist, self.speed_cb)
        if(self.use_reactive_input and not req.set_awareness.data): # if switching from REACTIVE manual to RAW manual control
            self.reactive_sub.unregister()
            self.tmux_sub = rospy.Subscriber(self.tmux_vel_topic, Twist, self.speed_cb)

        self.use_reactive_input = req.set_awareness.data
        #print('Awareness is: ', self.use_reactive_input)
        res = RobotAwarenessResponse()
        res.status.data = True
        return res

    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate): #compute max vel step
        step = ramp_rate * (t_now - t_prev).to_sec()
        sign = 1.0 if(v_target > v_prev) else -1.0
        error = math.fabs(v_target - v_prev)
        if (error < step): # if we can reach the target vel in the next step
            return v_target
        else:
            return (v_prev + sign * step)

    def ramped_twist(self, t_now): # split into component velocities
        tw = Twist()
        tw.angular.z = self.ramped_vel(self.last_twist.angular.z, self.target_twist.angular.z, 
                                    self.last_twist_send_time, t_now, self.vel_ramps[0])
        tw.linear.x = self.ramped_vel(self.last_twist.linear.x, self.target_twist.linear.x, 
                                        self.last_twist_send_time, t_now, self.vel_ramps[1])
        return tw

    def execute(self):
        t_now = rospy.Time.now()
        self.last_twist = self.ramped_twist(t_now)
        self.last_twist_send_time = t_now
        self.output_vel_pub.publish(self.last_twist)

    def speed_cb(self, msg): # set target velocity
        self.target_twist.angular.z = msg.angular.z * self.vel_scales[0]
        self.target_twist.linear.x = msg.linear.x * self.vel_scales[1]

    def fetch_param(self, name, default):
        if (rospy.has_param(name)):
            return rospy.get_param(name)
        else:
            rospy.logwarn("Parameter [%s] not defined. Defaulting to %.3f" % (name, default))
            return default

if __name__ == '__main__':
    rospy.init_node('speed_ramp_node')
    speed_ctl = SpeedRamp()

    rate = rospy.Rate(40)
    rospy.loginfo('Starting speed-ramp node ...')
    try:
        while (not rospy.is_shutdown()):
            speed_ctl.execute()
            rate.sleep()
    except rospy.exceptions.ROSInterruptException as ex:
        pass
    
