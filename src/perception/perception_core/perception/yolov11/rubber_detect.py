#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
import numpy as np
import pandas as pd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
import time
import cv2

from numpy import random
from perception.yolov11.models.experimental import attempt_load
from perception.yolov11.utils.datasets import letterbox
from perception.yolov11.utils.general import check_img_size, check_requirements, non_max_suppression, scale_coords
from perception.yolov11.utils.plots import plot_one_box
from perception.yolov11.utils.torch_utils import select_device, time_synchronized

import sys
import perception.yolov11.models as models
sys.modules['models'] = models
import perception.yolov11.utils as utils
sys.modules['utils'] = utils

# # ============================================================
# # Soft-NMS(DIoU) & 큰 박스 내부 2분할 유틸 (학습 없이 겹침 대응)
# # ============================================================
# def _diou_1vN(box, boxes):
#     """DIoU 유사 점수 (0~1로 clip)"""
#     x1 = np.maximum(box[0], boxes[:, 0])
#     y1 = np.maximum(box[1], boxes[:, 1])
#     x2 = np.minimum(box[2], boxes[:, 2])
#     y2 = np.minimum(box[3], boxes[:, 3])
#     inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)

#     area1 = (box[2] - box[0]) * (box[3] - box[1])
#     area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
#     union = area1 + area2 - inter + 1e-9
#     iou = inter / union

#     cx1 = (box[0] + box[2]) / 2.0
#     cy1 = (box[1] + box[3]) / 2.0
#     cx2 = (boxes[:, 0] + boxes[:, 2]) / 2.0
#     cy2 = (boxes[:, 1] + boxes[:, 3]) / 2.0
#     center_dist = (cx1 - cx2) ** 2 + (cy1 - cy2) ** 2

#     ex1 = np.minimum(box[0], boxes[:, 0])
#     ey1 = np.minimum(box[1], boxes[:, 1])
#     ex2 = np.maximum(box[2], boxes[:, 2])
#     ey2 = np.maximum(box[3], boxes[:, 3])
#     diag = (ex2 - ex1) ** 2 + (ey2 - ey1) ** 2 + 1e-9

#     diou = iou - center_dist / diag
#     return np.clip(diou, 0.0, 1.0)


# def soft_nms_diou(xyxy, scores, classes, sigma=0.5, score_thr=0.05):
#     """
#     Gaussian Soft-NMS with DIoU 감쇠.
#     입력:
#       - xyxy: (N,4) ndarray
#       - scores: (N,) ndarray
#       - classes: (N,) ndarray
#     출력:
#       - kept_xyxy, kept_scores, kept_classes
#     """
#     xyxy = xyxy.copy()
#     scores = scores.copy().astype(np.float32)
#     classes = classes.copy()

#     kept_b, kept_s, kept_c = [], [], []

#     while xyxy.shape[0] > 0:
#         m = int(np.argmax(scores))
#         b_m = xyxy[m].copy()
#         s_m = float(scores[m])
#         c_m = classes[m]

#         kept_b.append(b_m)
#         kept_s.append(s_m)
#         kept_c.append(c_m)

#         xyxy = np.delete(xyxy, m, axis=0)
#         scores = np.delete(scores, m, axis=0)
#         classes = np.delete(classes, m, axis=0)
#         if xyxy.shape[0] == 0:
#             break

#         dious = _diou_1vN(b_m, xyxy)
#         decay = np.exp(-(dious ** 2) / sigma)  # Gaussian decay
#         scores = scores * decay

#         keep_mask = scores > score_thr
#         xyxy = xyxy[keep_mask]
#         scores = scores[keep_mask]
#         classes = classes[keep_mask]

#     if len(kept_b) == 0:
#         return np.empty((0, 4), dtype=np.float32), np.empty((0,), dtype=np.float32), np.empty((0,), dtype=np.int32)

#     return (
#         np.array(kept_b, dtype=np.float32),
#         np.array(kept_s, dtype=np.float32),
#         np.array(kept_c, dtype=np.int32),
#     )


# def split_cone_bbox_by_color(img_bgr, bbox, min_gap_px=8):
#     """
#     한 덩어리로 나온 큰 상자 내부에서 주황(라바콘) 마스크의 세로 프로파일 valley를 찾아
#     좌/우 두 상자로 분할. 실패 시 원상자 반환.
#     """
#     try:
#         x1, y1, x2, y2 = map(int, bbox)
#         H, W = img_bgr.shape[:2]
#         x1 = max(0, min(x1, W - 1))
#         x2 = max(x1 + 1, min(x2, W))  # x2가 x1보다 커야 함
#         y1 = max(0, min(y1, H - 1))
#         y2 = max(y1 + 1, min(y2, H))  # y2가 y1보다 커야 함
        
#         if x2 - x1 < 10 or y2 - y1 < 10:
#             return [bbox]

#         crop = img_bgr[y1:y2, x1:x2]
#         if crop.size == 0:
#             return [bbox]

#         hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

#         # 조명/카메라에 맞춰 미세 조정 가능
#         lower = np.array([5, 80, 80], dtype=np.uint8)
#         upper = np.array([25, 255, 255], dtype=np.uint8)
#         mask = cv2.inRange(hsv, lower, upper)

#         proj = mask.sum(axis=0).astype(np.float32)
#         if proj.size < 24:
#             return [bbox]

#         k = max(3, proj.size // 50)
#         proj_s = cv2.blur(proj.reshape(1, -1), (1, k)).flatten()

#         mid = proj_s.size // 2
#         win = max(10, proj_s.size // 5)
#         s, e = max(0, mid - win), min(proj_s.size - 1, mid + win)
#         valley = s + int(np.argmin(proj_s[s:e]))

#         left_peak = float(proj_s[:valley].max() if valley > 0 else 0.0)
#         right_peak = float(proj_s[valley + 1:].max() if valley < proj_s.size - 1 else 0.0)
#         valley_val = float(proj_s[valley])

#         # 양쪽 peak가 있고, valley가 충분히 낮으면 분할 승인
#         if min(left_peak, right_peak) > 0 and valley_val < 0.35 * max(left_peak, right_peak):
#             if valley > min_gap_px and (proj_s.size - valley) > min_gap_px:
#                 x_mid = x1 + valley
#                 return [(x1, y1, x_mid, y2), (x_mid, y1, x2, y2)]

#         return [bbox]
    
#     except Exception as e:
#         print(f"Error in split_cone_bbox_by_color: {e}")
#         return [bbox]

# 하이퍼파라미터 및 설정
package_share = get_package_share_directory('perception')
WEIGHTS = os.path.join(package_share, 'yolov11', 'weights', 'rubber_ver2.pt')
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.40       # 0.60 → 0.25 (부분가림 상자 보존, 검출단계)
# POST_CONF = 0.60        # 최종 검출 결과 신뢰도
IOU_THRES = 0.80        # 0.35 → 0.80 (겹친 상자 동시 보존 유리)
CLASSES = None
AGNOSTIC_NMS = False

class YOLO(Node):
    def __init__(self):
        super().__init__('rubber_track')
        
        # 이미지 메시지를 구독할 서브스크라이버 생성
        # self.subscription = self.create_subscription(Image, '/usb_cam_1/image_raw', self.image_callback, 10)
        # self.subscription = self.create_subscription(Image, '/image_right', self.image_callback, 10)
        self.subscription = self.create_subscription(Image, '/image_combined', self.image_callback, 10)
        self.subscription  # 사용하지 않는 변수 경고 방지

        # PoseArray 메시지를 퍼블리시할 퍼블리셔 생성
        self.pose_array_pub = self.create_publisher(PoseArray, '/bounding_boxes/rubber', 10)
        self.img_res_pub = self.create_publisher(Image, '/yolo/rubber', 10)
        
        # CvBridge 초기화 (ROS 이미지와 OpenCV 이미지 간 변환)
        self.bridge = CvBridge()

        # YOLO 모델 초기화: 장치 선택, 모델 로딩, 이미지 사이즈 등 설정
        self.device = select_device(DEVICE)
        self.half = self.device.type != 'cpu'  # GPU 사용 시 half precision 적용
        self.get_logger().info(f"Using device: {self.device}")

        # 모델 로딩
        self.model = attempt_load(WEIGHTS, map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.imgsz = check_img_size(IMG_SIZE, s=self.stride)
        if self.half:
            self.model.half()  # FP16 변환

        # 모델이 검출할 클래스 이름과 각 클래스별 랜덤 색상 설정
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # GPU를 사용할 경우 모델 워밍업
        if self.device.type != 'cpu':
            dummy_input = torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters()))
            self.model(dummy_input)
        
        # 필수 패키지 요구사항 확인
        check_requirements(exclude=('pycocotools', 'thop'))

        self.get_logger().info("YOLO Detector node has been started.")

    def dominant_color(self, x, img): # 색 구분
        try:
            # 바운딩 박스 좌표 추출 및 유효성 검사
            xmin, ymin, xmax, ymax = int(x[0]), int(x[1]), int(x[2]), int(x[3])
            img_height, img_width = img.shape[:2]
            
            # 바운딩 박스가 이미지 범위 내에 있는지 확인
            xmin = max(0, min(xmin, img_width - 1))
            ymin = max(0, min(ymin, img_height - 1))
            xmax = max(xmin + 1, min(xmax, img_width))
            ymax = max(ymin + 1, min(ymax, img_height))
            
            # 바운딩 박스 크기가 너무 작은지 확인
            if (xmax - xmin) < 3 or (ymax - ymin) < 3:
                return 'unknown', [50, 50, 50], 0
            
            # 중간 y 좌표 계산 (바운딩 박스 아래쪽 절반만 사용)
            mid_y = int((ymin + ymax) / 2)
            # 바운딩 박스에서 색상 분석할 영역 추출
            box = img[mid_y:ymax, xmin:xmax]
            # 빈 배열 체크
            if box.size == 0:
                return 'unknown', [50, 50, 50], 0
            # 데이터 reshape 및 형변환
            data = np.reshape(box, (-1, 3))
            # 최소 픽셀 수 확인 (K-means를 위해)
            if len(data) < 1:
                return 'unknown', [50, 50, 50], 0
            data = np.float32(data)

            # K-means 클러스터링
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            flags = cv2.KMEANS_RANDOM_CENTERS
            compactness, labels, centers = cv2.kmeans(data, 1, None, criteria, 10, flags)

            # 주요 색상 추출
            dominant = centers[0].astype(np.int32)
            bgr = np.uint8([[dominant]])
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            h = hsv[0, 0, 0]
            colors = {'red': [0, 0, 255], 'yellow': [0, 255, 255], 'green': [0, 255, 0],
                     'blue': [255, 0, 0], 'unknown': [50, 50, 50]}
            
            if h < 16:
                color = 'red'
            elif h < 35:
                color = 'yellow'
            elif h < 92:
                color = 'green'
            elif h < 130:
                color = 'blue'
            elif h < 172:
                color = 'unknown'
            else:
                color = 'red'

            return color, colors[color], h
            
        except Exception as e:
            # 모든 예외 상황에서 기본값 반환
            self.get_logger().warn(f"dominant_color error: {e}")
            return 'unknown', [50, 50, 50], 0

    def image_callback(self, image_msg):
        with torch.no_grad():
            start_time = time.perf_counter()
            
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"CV Bridge error: {e}")
                return
            
            # 객체 검출 수행
            rubbers = self.detect(cv_image)
            if rubbers is None:
                return

            # for r in rubbers:
            #     _, x1, y1, x2, y2, conf = r
            #     cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            
            # 결과 이미지를 ROS 이미지 메시지로 변환 후 퍼블리시
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_message.header.stamp = self.get_clock().now().to_msg()
            self.img_res_pub.publish(image_message)
            
            # 검출 결과를 PoseArray 메시지로 구성
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'yolo'
            
            for rubber in rubbers:
                # if float(rubber[5]) < POST_CONF: # 후처리 신뢰도 임계값
                #     continue

                pose = Pose()
                # 각 필드에 검출 결과 할당 (클래스, 바운딩 박스 좌표, 신뢰도)
                pose.position.x = float(rubber[0])  # 클래스 인덱스 # 0 blue or 1 yellow, 2 other
                pose.position.y = float(rubber[5])  # 신뢰도
                # pose.position.z 는 0으로 유지됨
                pose.orientation.x = float(rubber[1])  # xmin
                pose.orientation.y = float(rubber[2])  # ymin
                pose.orientation.z = float(rubber[3])  # xmax
                pose.orientation.w = float(rubber[4])  # ymax

                # 클래스 인덱스를 색상 이름으로 변환
                color_names = {0: 'blue', 1: 'yellow', 2: 'other'}
                color_name = color_names.get(int(pose.position.x), 'unknown')
                
                self.get_logger().info(
                    f"({color_name}, reliability)=({pose.position.x:.1f}, {pose.position.y:.2f}) "
                    f"(xmin, ymin)=({pose.orientation.x:.0f}, {pose.orientation.y:.0f}) "
                    f"(xmax, ymax)=({pose.orientation.z:.0f}, {pose.orientation.w:.0f})"
                )
                
                pose_array.poses.append(pose)
            
            # 검출 결과 퍼블리시
            self.pose_array_pub.publish(pose_array)
            
            elapsed_time = time.perf_counter() - start_time
            self.get_logger().info(f"YOLO detection time: {elapsed_time:.5f} seconds")
    
    def detect(self, img0):
        # 이미지 전처리: letterbox로 크기 조정 및 패딩 추가
        img = letterbox(img0, self.imgsz, stride=self.stride)[0]
        # BGR → RGB 변환, 차원 순서 변경 및 메모리 연속성 확보
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)

        # 이미지를 torch 텐서로 변환
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # 데이터 타입 변환
        img /= 255.0  # 0~255 범위를 0~1 범위로 정규화
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        # 추론 수행
        pred = self.model(img, augment=AUGMENT)[0]
        
        # Non-Maximum Suppression 적용
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=CLASSES, agnostic=AGNOSTIC_NMS)
        
        det = pred[0]

        # if det is not None and len(det):
        #     # 원본 크기로 좌표 스케일 복원
        #     det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

        #     det_np = det.detach().cpu().numpy()
        #     xyxy = det_np[:, :4]                      # (N,4)
        #     scores = det_np[:, 4].astype(np.float32)  # (N,)
        #     classes = det_np[:, 5].astype(np.int32)   # (N,)

        #     # ---------- Soft-NMS(DIoU): 지우지 말고 점수 감쇠 ----------
        #     xyxy_s, scores_s, classes_s = soft_nms_diou(
        #         xyxy, scores, classes, sigma=0.5, score_thr=0.05
        #     )

        #     # ---------- 큰 박스 2분할(색 기반) + 한 번 더 약하게 Soft-NMS ----------
        #     rubbers = []
        #     if xyxy_s.shape[0] > 0:
        #         widths = (xyxy_s[:, 2] - xyxy_s[:, 0])
        #         median_w = float(np.median(widths)) if widths.size else 0.0
        #         big_thresh = 1.8 * median_w if median_w > 0 else float('inf')

        #         refined_boxes, refined_scores, refined_classes = [], [], []
        #         for b, s, c in zip(xyxy_s, scores_s, classes_s):
        #             w = b[2] - b[0]
        #             if w > big_thresh:
        #                 parts = split_cone_bbox_by_color(img0, b)
        #                 for pb in parts:
        #                     refined_boxes.append(pb)
        #                     refined_scores.append(s)
        #                     refined_classes.append(c)
        #             else:
        #                 refined_boxes.append(tuple(map(int, b)))
        #                 refined_scores.append(float(s))
        #                 refined_classes.append(int(c))

        #         xyxy_r = np.array(refined_boxes, dtype=np.float32)
        #         scores_r = np.array(refined_scores, dtype=np.float32)
        #         classes_r = np.array(refined_classes, dtype=np.int32)

        #         if xyxy_r.shape[0] > 0:
        #             xyxy_f, scores_f, classes_f = soft_nms_diou(
        #                 xyxy_r, scores_r, classes_r, sigma=0.5, score_thr=0.05
        #             )
        #         else:
        #             xyxy_f, scores_f, classes_f = xyxy_r, scores_r, classes_r

        #         # ---------- 색 재분류(blue=0 / yellow=1 / other=2) ----------
        #         for b, s in zip(xyxy_f.astype(int), scores_f):
        #             xmin, ymin, xmax, ymax = map(int, b.tolist())
        #             try:
        #                 color_name, color_bgr, hue = self.dominant_color([xmin, ymin, xmax, ymax], img0)
        #                 if color_name == 'blue':
        #                     new_cls = 0
        #                 elif color_name == 'yellow':
        #                     new_cls = 1
        #                 else:
        #                     new_cls = 2
        #             except Exception as e:
        #                 self.get_logger().warn(f"Color detection failed: {e}, using default class 2")
        #                 new_cls = 2

        #             rubbers.append([new_cls, xmin, ymin, xmax, ymax, float(s)])

        #     return rubbers

        # return None

        if det is not None and len(det):
            # 검출된 바운딩 박스 좌표를 원본 이미지 크기에 맞게 스케일 조정
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            rubbers = []
            for *xyxy, conf, cls in reversed(det):
                xmin, ymin, xmax, ymax = [int(tensor.item()) for tensor in xyxy]
                
                # 색상 분석을 통한 새로운 클래스 인덱스 부여
                try:
                    color_name, color_bgr, hue = self.dominant_color([xmin, ymin, xmax, ymax], img0)
                    # 파란색: 0, 노란색: 1, 나머지: 2
                    if color_name == 'blue':
                        new_cls = 0
                    elif color_name == 'yellow':
                        new_cls = 1
                    else:
                        new_cls = 2
                    
                    self.get_logger().debug(f"Detected color: {color_name} (hue: {hue}) -> class: {new_cls}")
                except Exception as e:
                    self.get_logger().warn(f"Color detection failed: {e}, using default class 2")
                    new_cls = 2
                
                rubber = [new_cls, xmin, ymin, xmax, ymax, conf]
                rubbers.append(rubber)
            return rubbers
        return None

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLO()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
