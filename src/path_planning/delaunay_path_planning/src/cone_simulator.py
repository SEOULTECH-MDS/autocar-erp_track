#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ConeSimulator(Node):
    def __init__(self):
        super().__init__('cone_simulator')
        
        # Publisher
        self.cone_pub = self.create_publisher(MarkerArray, '/sensor_fusion/tracked_rubber_cones', 10)
        
        # Timer - 10Hz로 콘 데이터 발행
        self.timer = self.create_timer(0.1, self.publish_cones)
        
        # 가상 트랙 파라미터
        self.track_width = 4.0  # 트랙 폭 (m)
        self.cone_spacing = 3.0  # 콘 간격 (m)
        self.track_length = 50.0  # 트랙 길이 (m)
        
        self.get_logger().info('Cone Simulator 노드가 시작되었습니다.')
    
    def create_straight_track(self):
        """직선 트랙의 콘들을 생성"""
        blue_cones = []  # 왼쪽 콘 (파란색)
        yellow_cones = []  # 오른쪽 콘 (노란색)
        
        # 직선 트랙 생성
        for x in np.arange(2.0, self.track_length, self.cone_spacing):
            # 왼쪽 콘 (파란색) - y = +track_width/2
            blue_cones.append([x, self.track_width/2])
            # 오른쪽 콘 (노란색) - y = -track_width/2  
            yellow_cones.append([x, -self.track_width/2])
        
        return blue_cones, yellow_cones
    
    def create_curved_track(self):
        """곡선 트랙의 콘들을 생성"""
        blue_cones = []  # 왼쪽 콘 (파란색)
        yellow_cones = []  # 오른쪽 콘 (노란색)
        
        # S자 곡선 트랙 생성
        for i, x in enumerate(np.arange(2.0, self.track_length, self.cone_spacing)):
            # S자 곡선 함수
            curve_offset = 3.0 * math.sin(x * 0.2)  # 진폭 3m, 주기 조절
            
            # 왼쪽 콘 (파란색)
            blue_y = self.track_width/2 + curve_offset
            blue_cones.append([x, blue_y])
            
            # 오른쪽 콘 (노란색)
            yellow_y = -self.track_width/2 + curve_offset
            yellow_cones.append([x, yellow_y])
        
        return blue_cones, yellow_cones
    
    def create_oval_track(self):
        """타원형 트랙의 콘들을 생성"""
        blue_cones = []  # 안쪽 콘 (파란색)
        yellow_cones = []  # 바깥쪽 콘 (노란색)
        
        # 타원 파라미터
        a = 15.0  # 장축 반지름
        b = 8.0   # 단축 반지름
        center_x, center_y = 20.0, 0.0
        
        # 타원 경로를 따라 콘 배치
        for theta in np.arange(0, 2*math.pi, 0.3):  # 약 0.3 라디안 간격
            # 타원 중심선
            x_center = center_x + a * math.cos(theta)
            y_center = center_y + b * math.sin(theta)
            
            # 법선 벡터 계산 (타원에 수직인 방향)
            normal_x = -a * math.sin(theta)
            normal_y = b * math.cos(theta)
            normal_length = math.sqrt(normal_x**2 + normal_y**2)
            normal_x /= normal_length
            normal_y /= normal_length
            
            # 안쪽 콘 (파란색)
            inner_x = x_center - normal_x * (self.track_width/2)
            inner_y = y_center - normal_y * (self.track_width/2)
            blue_cones.append([inner_x, inner_y])
            
            # 바깥쪽 콘 (노란색)
            outer_x = x_center + normal_x * (self.track_width/2)
            outer_y = y_center + normal_y * (self.track_width/2)
            yellow_cones.append([outer_x, outer_y])
        
        return blue_cones, yellow_cones
    
    def create_marker(self, cone_positions, color, marker_id):
        """콘 위치들로부터 Marker 생성"""
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cones"
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # 크기 설정
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 색상 설정
        marker.color.a = 1.0
        if color == "blue":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        elif color == "yellow":
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        
        # 점들 추가
        for cone_pos in cone_positions:
            point = Point()
            point.x = float(cone_pos[0])
            point.y = float(cone_pos[1])
            point.z = 0.0
            marker.points.append(point)
        
        return marker
    
    def publish_cones(self):
        """콘 데이터를 발행"""
        marker_array = MarkerArray()
        
        # 트랙 타입 선택 (원하는 트랙으로 변경 가능)
        # blue_cones, yellow_cones = self.create_straight_track()
        blue_cones, yellow_cones = self.create_curved_track()
        # blue_cones, yellow_cones = self.create_oval_track()
        
        # 노이즈 추가 (실제 센서 데이터와 유사하게)
        noise_level = 0.1  # 10cm 노이즈
        for cone in blue_cones:
            cone[0] += np.random.normal(0, noise_level)
            cone[1] += np.random.normal(0, noise_level)
        
        for cone in yellow_cones:
            cone[0] += np.random.normal(0, noise_level)
            cone[1] += np.random.normal(0, noise_level)
        
        # 파란색 콘 마커 생성
        if blue_cones:
            blue_marker = self.create_marker(blue_cones, "blue", 0)
            marker_array.markers.append(blue_marker)
        
        # 노란색 콘 마커 생성
        if yellow_cones:
            yellow_marker = self.create_marker(yellow_cones, "yellow", 1)
            marker_array.markers.append(yellow_marker)
        
        # 발행
        self.cone_pub.publish(marker_array)
        
        # 로그 출력 (너무 많이 출력되지 않도록 가끔만)
        if len(marker_array.markers) > 0:
            total_cones = len(blue_cones) + len(yellow_cones)
            self.get_logger().info(f'발행된 콘 개수: 파란색={len(blue_cones)}, 노란색={len(yellow_cones)}, 총={total_cones}개', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    
    cone_simulator = ConeSimulator()
    
    try:
        rclpy.spin(cone_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        cone_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
