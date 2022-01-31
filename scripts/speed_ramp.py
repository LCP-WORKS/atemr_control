#! /usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from atemr_msgs.srv import RobotAwareness, RobotAwarenessResponse
from atemr_msgs.msg import SpeedRampStatus


class SpeedRamp:
    def __init__(self, tmux_vel_topic="tmux/cmd_vel", reactive_vel_topic="reactive/cmd_vel", out_vel_topic="base_controller/cmd_vel"):
        self.last_twist_send_time = self.twist_idle_time = rospy.Time.now()
        self.awareness_srvr = rospy.Service('RobotAwarenessServer', RobotAwareness, self.awareness_cb)
        self.output_vel_pub = rospy.Publisher(out_vel_topic, Twist, queue_size=1)

        self.use_reactive_input = False
        self.tmux_vel_topic = tmux_vel_topic
        self.reactive_vel_topic = reactive_vel_topic
        self.tmux_sub = rospy.Subscriber(self.tmux_vel_topic, Twist, self.speed_cb)

        self.emergency_timer = None
        self.is_idle = True
        self.OBSTACLE_STOP_DISTANCE = self.fetch_param('~obstacle_stop_distance', 0.28) #28cm
        self.OBSTACLE_SLOW_DISTANCE = self.fetch_param('~obstacle_slow_distance', 0.56) #56cm
        self.APPROACH_LINEAR_VELOCITY = self.fetch_param('~approach_linear_velocity', 0.35) #m/s
        self.APPROACH_ANGULAR_VELOCITY = self.fetch_param('~approach_angular_velocity', 0.6) #rad/s
        self.RATE_IDLE_TIME = self.fetch_param('~time_to_idle', 8.0) #secs

        self.obs_emergency = self.slow_down = self.idle_timer =  self.running_timer = False
        self.pattern_sub = rospy.Subscriber('/pattern', Float32MultiArray, self.pattern_cb)
        self.ramp_status_pub = rospy.Publisher('/speedramp_status', SpeedRampStatus, queue_size=1)
        self.ramp_status_msg = SpeedRampStatus()

        self.target_twist = Twist()
        self.last_twist = Twist()
        self.vel_scales = [0.1] * 2
        self.vel_scales[0] = self.fetch_param('~angular_scale', 0.1) # default to very slow
        self.vel_scales[1] = self.fetch_param('~linear_scale', 0.1)
        self.vel_ramps = [1.0] * 2
        self.vel_ramps[0] = self.fetch_param('~angular_accel', 1.0) # units: meter per second^2
        self.vel_ramps[1] = self.fetch_param('~linear_accel', 1.0)

    '''Oneshot emergency release'''
    def emergency_timer_cb(self, e):
        self.obs_emergency = False if(self.obs_emergency) else self.obs_emergency
        self.emergency_timer = None
    
    def pattern_cb(self, msg):
        if((len([ptn for ptn in msg.data if (ptn > 0.0 and ptn <= self.OBSTACLE_STOP_DISTANCE)]) != 0)):
            self.obs_emergency = True #activate emergency stop
        else:
            if((self.obs_emergency) and (self.emergency_timer == None)):
                self.emergency_timer = rospy.Timer(rospy.Duration(2), self.emergency_timer_cb, oneshot=True)
        if((len([ptn for ptn in msg.data if (ptn > self.OBSTACLE_STOP_DISTANCE and ptn <= self.OBSTACLE_SLOW_DISTANCE)]) == 0)):
            self.slow_down = False #deactivate slow_down
        else:
            if(not self.obs_emergency and (not self.slow_down)):
                self.slow_down = True #activate slow down
    
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
        self.target_twist.angular.z = 0.0
        self.target_twist.linear.x = 0.0
        
        #IDLE/ RUNNING detector
        if((abs(self.last_twist.angular.z) <= 0.0) and (abs(self.last_twist.linear.x) <= 0.0)):
            if((not self.idle_timer)):
                self.twist_idle_time = rospy.Time.now()
                self.idle_timer = True
                self.running_timer = False
            if(not self.is_idle):
                if((rospy.Time.now() - self.twist_idle_time).to_sec() > self.RATE_IDLE_TIME):
                    self.is_idle = True
        else:
            if(not self.running_timer):
                self.twist_idle_time = rospy.Time.now()
                self.running_timer = True
                self.idle_timer = False
            if(self.is_idle): #reset if valid command received for a certain time period
                if((rospy.Time.now() - self.twist_idle_time).to_sec() > (self.RATE_IDLE_TIME/2.0)):
                    self.is_idle = False
        
        self.ramp_status_msg.is_idle.data = self.is_idle
        self.ramp_status_msg.is_emergency.data = self.obs_emergency
        self.ramp_status_msg.is_approach.data = self.slow_down
        self.ramp_status_msg.is_aware.data = self.use_reactive_input
        self.ramp_status_pub.publish(self.ramp_status_msg)

    def speed_cb(self, msg): # set target velocity
        if((self.slow_down) and (not self.obs_emergency)):
            self.target_twist.angular.z = (self.APPROACH_ANGULAR_VELOCITY * self.vel_scales[0] * -1.0 if(msg.angular.z < 0) else 1.0)\
                 if(abs(msg.angular.z) > self.APPROACH_ANGULAR_VELOCITY) else (msg.angular.z * self.vel_scales[0])
            self.target_twist.linear.x = (self.APPROACH_LINEAR_VELOCITY * self.vel_scales[1] * -1.0 if(msg.linear.x < 0) else 1.0)\
                 if(abs(msg.linear.x) > self.APPROACH_LINEAR_VELOCITY) else (msg.linear.x * self.vel_scales[1])
        if(self.obs_emergency):
            self.target_twist.angular.z = 0.0
            self.target_twist.linear.x = 0.0
        if((not self.slow_down) and (not self.obs_emergency)):
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

    rate = rospy.Rate(50)
    rospy.loginfo('Starting speed-ramp node ...')
    try:
        while (not rospy.is_shutdown()):
            speed_ctl.execute()
            rate.sleep()
    except rospy.exceptions.ROSInterruptException as ex:
        pass
    
