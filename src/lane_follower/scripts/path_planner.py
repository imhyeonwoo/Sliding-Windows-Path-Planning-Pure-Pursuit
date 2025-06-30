#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1) lane_detector 가 보낸 좌/우 차선 Poly-fit 계수 수신
2) 차량-중앙 경로(P(x)) 계산
3) Look-ahead 점 (x_la,y_la) 을 /path/lookahead_point 로 발행
   - geometry_msgs/Point32 (x=가로[미터], y=세로[미터])
"""
import rospy, numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point32

real_shift_distance = 0.425   # 차량 중심 ↔ 차선 간격 (미터)
x_la = 0.85                  # look-ahead 세로거리 (미터)

class PathPlanner:
    def __init__(self):
        rospy.init_node("path_planner")
        self.left_fit, self.right_fit = None, None

        rospy.Subscriber("/lane/left_fit",  Float32MultiArray,
                         lambda m: setattr(self, "left_fit",  m.data))
        rospy.Subscriber("/lane/right_fit", Float32MultiArray,
                         lambda m: setattr(self, "right_fit", m.data))

        self.pub_la = rospy.Publisher("/path/lookahead_point", Point32, queue_size=1)
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.loop(); self.rate.sleep()

    def loop(self):
        p = Point32(); p.z = 0.0
        # 오른쪽 차선이 우선 (3차식)
        if self.right_fit:
            a,b,c,d = self.right_fit
            R_prime = 3*a*x_la**2 + 2*b*x_la + c
            theta = np.arctan(R_prime)
            y_shift =  real_shift_distance*np.cos(theta)
            x_shift = -real_shift_distance*np.sin(theta)
            p.y = x_la                      # 로컬 좌표계: y = 전방거리
            p.x = (a*p.y**3 + b*p.y**2 + c*p.y + d) + y_shift + x_shift*0
            self.pub_la.publish(p)
        # 왼쪽 차선만 있는 경우 (1차식)
        elif self.left_fit:
            m, b = self.left_fit
            p.y = x_la
            p.x = (m*p.y + b) - real_shift_distance
            self.pub_la.publish(p)

if __name__ == "__main__":
    PathPlanner()
