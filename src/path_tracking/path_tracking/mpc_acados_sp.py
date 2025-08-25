#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from autocar_utils.euler_from_quaternion import euler_from_quaternion
from autocar_utils.yaw_to_quaternion import yaw_to_quaternion
from autocar_utils.normalise_angle import normalise_angle
from .acados_setting_sp import acados_solver
from autocar_utils.utils import CubicSpline2D
# from sensor_msgs.msg import TwistWithCovarianceStamped

from rviz_2d_overlay_msgs.msg import OverlayText
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy

NX = 5  # 상태 변수 크기 (x, y, yaw, v, s)
NU = 2 # 제어 입력 크기 (delta , a)
T = 2.0  # 예측 시간 [s]
N = 20  # 예측 구간 [s]

class Control(Node):
    def __init__(self):
        super().__init__('control')

        # Publisher
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        # 시각화 Publisher
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/overlay', 10)
        self.mpc_predict_pub = self.create_publisher(MarkerArray, '/autocar/mpc_predict', qos_profile=qos_transient_local)
        self.mpc_ref_pub = self.create_publisher(MarkerArray, '/autocar/mpc_ref', qos_profile=qos_transient_local)

        # Subscriber
        # self.global_waypoints_sub = self.create_subscription(PoseArray, '/autocar/goals', self.global_waypoints_cb, 10)
        self.local_path_sub = self.create_subscription(Path, '/local_path', self.local_path_cb, 10)
        # self.speed_sub = self.create_subscription(TwistWithCovarianceStamped, "/ublox_gps/fix_velocity", self.speed_cb, 10)  # 차량 현재 속도

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None
        self.s = None

        self.xs = []
        self.ys = []
        self.cubic_spline= None  

        self.lock = threading.Lock()
        self.control_frequency = 20.0 # HZ
        self.dt = T / N

        self.target_vel = 4.0  # 목표 속도 (m/s)
        self.steering_angle = 0.0
        self.velocity = 0.0
        
        # 이전 제어 입력 저장용 변수 (solver 실패 시 fallback용)
        self.prev_steering_angle = 0.0
        self.prev_velocity = 0.0
        self.fail_count = 0  # 실패 횟수 카운트
        
        # s 값 제약을 위한 변수들
        self.prev_s = 0.0  # 이전 s 값 저장
        self.s_tolerance = 30.0  # s 값이 역행할 수 있는 최대 허용 범위 (m)

        # MPC Solver 초기화
        self.solver = acados_solver() 

        # 주기적인 제어 실행을 위한 타이머 설정
        self.timer_control = self.create_timer(1.0 / self.control_frequency, self.mpc_control)


    def speed_cb(self, speed_msg):
        """
        차량 속도 콜백
        """
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = np.sqrt(speed_msg.twist.twist.linear.x **2 + speed_msg.twist.twist.linear.y **2)
        self.s = 0.0
        return
    
    def local_path_cb(self, path_msg):
        """
        local path 콜백
        """
        with self.lock:
            if len(path_msg.poses) < 2:
                self.get_logger().warn("local path가 충분하지 않습니다.")
                return
            
            self.xs = [pose.pose.position.x for pose in path_msg.poses]
            self.ys = [pose.pose.position.y for pose in path_msg.poses]
            self.cubic_spline = CubicSpline2D(self.xs, self.ys)

            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.v = 0.0
            self.s = 0.0
        return

    def calc_current_s(self, _cubic_spline):
        """
        이진 탐색과 gradient descent를 사용한 s 값 탐색
        최근 s값 기준으로 일정 범위 안에서만 탐색
        """
        cubic_spline = _cubic_spline

        if self.x is None or self.y is None:
            self.s = 0.0
            return
            
        if not hasattr(cubic_spline, 's') or len(cubic_spline.s) == 0:
            self.s = 0.0
            return
        
        total_length = cubic_spline.s[-1]
        
        # s 탐색 범위를 이전 s 값 기준으로 제한
        search_start = max(0, self.prev_s - self.s_tolerance)
        search_end = min(total_length, self.prev_s + 50.0)  # 앞으로 최대 50m까지만 탐색
        
        # 1단계: 제한된 범위에서 거친 탐색
        coarse_resolution = 1.0  # 1m 간격
        s_coarse = np.arange(search_start, search_end + coarse_resolution, coarse_resolution)
        
        min_distance = float('inf')
        best_s = self.prev_s  # 기본값을 이전 s로 설정
        
        for s_val in s_coarse:
            sx, sy = cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val
        
        # 2단계: 최적 지점 주변을 세밀하게 탐색
        search_range = 2.0  # 최적점 주변 ±2m 범위
        s_start = max(search_start, best_s - search_range)
        s_end = min(search_end, best_s + search_range)
        
        fine_resolution = 0.05  # 0.05m 간격으로 세밀 탐색
        s_fine = np.arange(s_start, s_end + fine_resolution, fine_resolution)
        
        for s_val in s_fine:
            sx, sy = cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val
        
        # s 값이 너무 크게 역행하는 것을 방지 
        if best_s < self.prev_s - self.s_tolerance:
            best_s = self.prev_s - self.s_tolerance
            self.get_logger().warn(f"S 값 역행 제한: {best_s:.2f} (이전: {self.prev_s:.2f})")
        
        self.prev_s = best_s  # 이전 s 값 업데이트
        self.s = best_s
        return




    def calc_ref_trajectory(self, _cubic_spline):
        """
        MPC 예측 step에 대한 refrecne trajectory 계산
        """
        cubic_spline = _cubic_spline

        xref = np.zeros((NX, N)) # reference x, y, yaw, v, s
        tan_vec = np.zeros((2, N)) # 접선 벡터 tx, ty

        if cubic_spline:
            current_s = self.s

            for i in range(N):
                # 다음 s 값을 먼저 계산
                s = current_s + self.dt * self.target_vel
                
                # s가 끝점을 넘어갔는지 확인
                if s > cubic_spline.s[-1]:
                    # 끝점을 넘어갔으면 끝점으로 고정하고 속도 0 설정
                    s = cubic_spline.s[-1] - 0.1
                    target_vel = 0.0
                else:
                    # 끝점까지 남은 거리 계산
                    remaining_distance = cubic_spline.s[-1] - s

                    if remaining_distance <= 3.0:  # 3m 이내에서 미리 속도 0 설정
                        target_vel = 0.0
                    else:
                        # 정상 범위 내라면 원래 목표 속도 사용
                        target_vel = self.target_vel
                
                # 다음 iteration을 위해 current_s 업데이트
                current_s = s

                xref[0, i], xref[1, i] = cubic_spline.calc_position(s)
                xref[2, i] = cubic_spline.calc_yaw(s)
                xref[3, i] = target_vel
                xref[4, i] = s 

                tan_vec[0, i] = math.cos(cubic_spline.calc_yaw(s))
                tan_vec[1, i] = math.sin(cubic_spline.calc_yaw(s))

        self.visualize_ref_trajectory(xref)

        return xref, tan_vec

    def mpc_control(self):
        """
        MPC 제어 루프
        """

        if self.x is None or self.y is None or self.yaw is None or self.v is None:
            self.get_logger().warn("차량 상태가 초기화되지 않았습니다.")
            return

        if self.cubic_spline is None:
            self.get_logger().warn("Cubic spline이 초기화되지 않았습니다.")
            return
        
        current_cubic_spline = self.cubic_spline

        # 현재 s 값 계산, reference trajectory 계산
        self.calc_current_s(current_cubic_spline)
        xref, tan_vec = self.calc_ref_trajectory(current_cubic_spline)
        x0 = np.array([self.x, self.y, self.yaw, self.v, self.s])

        u_opt = np.zeros((N, NU))  # 제어 입력 초기화 (delta, a)
        x_opt = np.zeros((N, NX))  # 상태 변수 초기화 (x, y, yaw, v, s)

        # Solver 초기 상태 변수 설정 
        self.solver.set(0, "x", x0)
        self.solver.constraints_set(0, "lbx", x0)
        self.solver.constraints_set(0, "ubx", x0)
        self.get_logger().info(f"Current state: {x0}")

        # MPC Solver에 파라미터 변수 전달
        for i in range(N):
            self.solver.set(i, "p", np.hstack([xref[:5, i], u_opt[i, 0] ,tan_vec[:, i] ]))
        self.solver.set(N, "p", np.hstack([xref[:5, -1], u_opt[-1, 0], tan_vec[:, -1] ]))

        # Solver 실행, status 확인
        status = self.solver.solve()
        if status != 0:
            self.fail_count += 1
            self.get_logger().error(f"MPC Solver failed with status {status}.")
            # self.prev_steering_angle *= 0.98
            self.prev_velocity *= 0.98  # solver failure 시 속도 감소
            self.set_vehicle_command(self.prev_steering_angle, self.prev_velocity) # 이전 제어 입력으로 차량에 입력
            return
        
        # self.get_logger().info(f"tan_vec: {tan_vec}\
        #                        \n xref: {xref[:, 0]}, {xref[:, 1]}, {xref[:, 2]}, {xref[:, 3]}, {xref[:, 4]}")
        
        # Solver에서 최적화된 제어 입력, 상태 변수 추출
        u_opt = np.array([self.solver.get(i, "u") for i in range(N)])
        x_opt = np.array([self.solver.get(i, "x") for i in range(N)])
        self.visualize_predicted_trajectory(x_opt)

        # 제어 입력
        # self.steering_angle = u_opt[1, 0] - 0.14137166941  # 조향각 (delta) alignment 보정 -2.7도
        self.steering_angle = u_opt[1, 0]  # 조향각 (delta)
        self.velocity = x_opt[1, 3]        # 속도 (v)

        # 이전 제어 입력 저장 (다음 실패 시 fallback용)
        self.prev_steering_angle = self.steering_angle
        self.prev_velocity = self.velocity

        # s 값이 목표 지점에 도달했는지 확인 -> local path 활용 시 s의 끝에 도달했을 떄 속도를 0으로 설정
        remaining_distance = current_cubic_spline.s[-1] - self.s
        if remaining_distance <= 0.3:
            self.velocity = 0.0 # path의 끝에 도달했을 떄 속도를 0으로 설정 -> 브레이크

        # 차량에 제어 명령 전송
        self.set_vehicle_command(self.steering_angle, self.velocity)

        self.get_logger().info(f"cmd_steer: {self.steering_angle * 180.0 / np.pi:.2f} deg, cmd_vel: {self.velocity:.2f} m/s")

    def set_vehicle_command(self, steering_angle, velocity):
        """
        차량 명령 퍼블리시
        """

        cmd = AckermannDriveStamped()
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = steering_angle

        self.erp_pub.publish(cmd)

        self.publish_overlay_text()
        self.get_logger().info(f"속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg")



# _____________________________ visualization ______________________________#

    def publish_overlay_text(self):
        text_msg = OverlayText()
        text_msg.width = 500
        text_msg.height = 200
        text_msg.text_size = 15.0
        text_msg.line_width = 2

        text_msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5) # 배경색 (반투명 검정)
        text_msg.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # 글자색 (파란색)

        # 표시할 텍스트 설정
        text_msg.text = f"Velocity: {self.velocity:.2f}m/s \n Steer: {self.steering_angle * 180.0 / np.pi:.2f}deg\
            \n Fail Count: {self.fail_count}\
            \n Prev input: {self.prev_steering_angle * 180.0 / np.pi:.2f} deg, {self.prev_velocity:.2f} m/s \
            \n State: ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}, {self.v:.2f}, {self.s:.2f})"


        self.overlay_pub.publish(text_msg)

    def visualize_predicted_trajectory(self, x_opt):
        """
        Solver에서 예측된 경로를 시각화
        """
        marker_array = MarkerArray()

        for i in range(x_opt.shape[0]):  # x_opt의 각 점에 대해 반복
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "predicted_points"
            marker.id = i
            marker.type = Marker.ARROW  # 화살표로 표시
            marker.action = Marker.ADD

            # 위치 설정
            marker.pose.position.x = x_opt[i, 0]  # x 위치
            marker.pose.position.y = x_opt[i, 1]  # y 위치
            marker.pose.position.z = 0.0

            # 방향 설정 (yaw를 쿼터니언으로 변환)
            yaw = x_opt[i, 2]  # yaw 값
            quaternion = yaw_to_quaternion(yaw)
            marker.pose.orientation.x = quaternion.x
            marker.pose.orientation.y = quaternion.y
            marker.pose.orientation.z = quaternion.z
            marker.pose.orientation.w = quaternion.w

            # 크기 설정
            marker.scale.x = 0.3  # 화살표 길이
            marker.scale.y = 0.05  # 화살표 두께
            marker.scale.z = 0.05  # 화살표 두께

            # 색상 설정
            marker.color.r = 0.0  
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_predict_pub.publish(marker_array)


    def visualize_ref_trajectory(self, xref):
        """
        xref를 시각화
        """
        marker_array = MarkerArray()

        for i in range(xref.shape[1]):  # xref의 각 점에 대해 반복
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "xref_points"
            marker.id = i
            marker.type = Marker.ARROW  # 화살표로 표시
            marker.action = Marker.ADD

            # 위치 설정
            marker.pose.position.x = xref[0, i]  # x 위치
            marker.pose.position.y = xref[1, i]  # y 위치
            marker.pose.position.z = 0.0

            # 방향 설정 (yaw를 쿼터니언으로 변환)
            yaw = xref[2, i]  # yaw 값
            quaternion = yaw_to_quaternion(yaw)
            marker.pose.orientation.x = quaternion.x
            marker.pose.orientation.y = quaternion.y
            marker.pose.orientation.z = quaternion.z
            marker.pose.orientation.w = quaternion.w

            # 크기 설정
            marker.scale.x = 0.3  # 화살표 길이
            marker.scale.y = 0.05  # 화살표 두께
            marker.scale.z = 0.05  # 화살표 두께

            # 색상 설정
            marker.color.r = 1.0
            marker.color.g = 1.0  # 초록색
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_ref_pub.publish(marker_array)

# _____________________________ main function ______________________________#
def main(args=None):
    rclpy.init(args=args)
    try:
        control = Control()
        rclpy.spin(control)
    finally:
        control.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()