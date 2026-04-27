#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, String

from duckietown_msgs.msg import Twist2DStamped
import os
from switch_control_node import ControlType
import yaml
import util

class ControlLaneNode:
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.enable = True

        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(detect_lane_topic, Float64, self.cbFollowLane, queue_size = 1)

        control_change_topic = f"/{self._vehicle_name}/switch/control"
        self.sub_control = rospy.Subscriber(control_change_topic, Int32, self.cbControl , queue_size = 1)
 
        self.lastError = 0
        self.v = 0
        self.a = 0

        self.integral = 0.0           
        self.last_time = None

        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self,msg):
        if msg.data == ControlType.Lane.value:
            self.enable = True
        
        else:
            self.enable = False

    def cbUpdateParameters(self,parameters):
        self.kp = parameters["pid"]["p"]["default"]
        self.ki = parameters["pid"]["i"]["default"]
        self.kd = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]

    # error between 1 and -1
    def cbFollowLane(self, error):
        print(f'received message. enabled : {self.enable}')
        error = error.data

        # Wenn der Node nicht aktiv ist, nichts tun
        if not self.enable:
            self.v = 0.0
            self.a = 0.0
            return

        # 1. Zeitdifferenz (dt) berechnen
        current_time = rospy.Time.now().to_sec()
        
        # Beim allerersten Aufruf haben wir noch kein dt
        if self.last_time is None:
            self.last_time = current_time
            self.lastError = error
            return
            
        dt = current_time - self.last_time
        
        # Schutz gegen Division durch Null, falls Nachrichten zu schnell kommen
        if dt <= 0.0:
            return

        # ==========================================
        # PID BERECHNUNG
        # ==========================================

        # P-Anteil (Proportional)
        p_term = self.kp * error

        # I-Anteil (Integral) - summiert den Fehler über die Zeit auf
        self.integral += error * dt
        
        # Anti-Windup: Begrenzt das Integral, damit es nicht explodiert
        max_integral = 1.0  # Kann bei Bedarf angepasst werden
        self.integral = max(min(self.integral, max_integral), -max_integral)
        
        i_term = self.ki * self.integral

        # D-Anteil (Derivative) - reagiert auf die Änderung des Fehlers
        d_term = self.kd * ((error - self.lastError) / dt)

        # Gesamter Output für die Lenkung (Winkelgeschwindigkeit omega)
        omega = p_term + i_term + d_term

        # ==========================================
        # GESCHWINDIGKEIT (LINEAR VELOCITY)
        # ==========================================
        # Dynamische Geschwindigkeit: Wenn der Fehler groß ist (Kurve), 
        # fahren wir langsamer. Wenn er 0 ist (Gerade), fahren wir MAX_VEL.
        # (Faktor 0.5 bedeutet, bei maximalem Error (1) fahren wir halbe Kraft)
        velocity = self.MAX_VEL * (1.0 - (abs(error) * 0.8))
        
        # Sicherheitslimit nach unten, damit er nicht stehenbleibt
        velocity = max(velocity, 0.04) 

        # ==========================================
        # WERTE FÜR DIE PUBLISH-SCHLEIFE SETZEN
        # ==========================================
        self.v = velocity
        self.a = omega   # self.a ist in deiner run() Schleife twist.omega

        # Zustand für den nächsten Aufruf speichern
        self.lastError = error
        self.last_time = current_time
        

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(f"{self.kp} {self.ki} {self.kd} {self.MAX_VEL}")
            if self.enable:
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                
                twist.v = self.v
                twist.omega = self.a
                self.pub_cmd_vel.publish(twist)

                rate.sleep()

if __name__ == '__main__':
    # create the node
    node = ControlLaneNode('control_lane_node')
    node.run()
    rospy.spin()