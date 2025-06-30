#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1) path_planner 가 주는 Look-ahead 점을 수신
2) Pure-Pursuit 식으로 조향각 계산 → /car/steering
3) 스로틀은 고정 0.8 → /car/throttle
"""
import rospy, numpy as np
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

L_m = 0.55            # wheel-base
throttle_cmd = 0.8    # 상수 스로틀

class PurePursuitCtrl:
    def __init__(self):
        rospy.init_node("pure_pursuit_ctrl")
        self.pub_steer   = rospy.Publisher("/car/steering", Float32, queue_size=1)
        self.pub_throttle= rospy.Publisher("/car/throttle", Float32, queue_size=1)

        rospy.Subscriber("/path/lookahead_point", Point32, self.cb_la, queue_size=1)
        rospy.spin()

    def cb_la(self, p):
        y_la, x_la = p.y, p.x
        steer = np.degrees(np.arctan2(2*L_m*y_la, x_la**2 + y_la**2))
        steer = float(np.clip(steer, -30.0, 30.0))
        self.pub_steer.publish(Float32(data=steer))
        self.pub_throttle.publish(Float32(data=throttle_cmd))
        rospy.loginfo_throttle(1.0, f"Steer={steer: .2f}°, Throttle={throttle_cmd}")

if __name__ == "__main__":
    PurePursuitCtrl()
