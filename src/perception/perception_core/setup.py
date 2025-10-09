from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/perception']),
        ('share/perception', ['package.xml']),

        # lanenet 모델 파일
        ('share/perception/models', ['perception/lanenet/models/culane_18.pth']),

        # ==============================
        # YOLO 가중치 파일 설치 부분 추가
        # perception/yolov11/weights/*.pt 파일을 모두 설치
        # ==============================
        (os.path.join('share', package_name, 'yolov11', 'weights'), glob('perception/yolov11/weights/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkny2003',
    maintainer_email='kkny2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 정지선
            'stopline_camera = perception.stopline_camera:main',
            'stopline_detection = perception.stopline_detection:main',

            # 차선
            'camera_pub = perception.lanenet.camera_pub:main',
            'lanenet = perception.lanenet.lanenet:main',

            # 라바콘 트랙 주행
            "left_camera = perception.yolov11.obstacle_camera.left_camera:main",
            "right_camera = perception.yolov11.obstacle_camera.right_camera:main",
            'combined_camera = perception.yolov11.obstacle_camera.combined_camera:main',
            'rubber_detect = perception.yolov11.rubber_detect:main',
            "sensor_fusion_rubber = perception.sensor_fusion.src.rubber.sensor_fusion:main",
            "bbox_tracker = perception.tracker.src.bbox_tracker:main",
            "object_tracker3D = perception.tracker.src.object_tracker3D:main",
            "rubber_visualizer = perception.rubber_visualizer.src.rubber_visualizer:main",
        ],
    },
)
