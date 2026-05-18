     1 import rclpy
     2 from rclpy.node import Node
     3 import math
     4 import time
     5# gpt-4.1 대신 표준 모델인 gpt-4o 사용
    temperature=0.1, # 창의성 낮추고, 일관된 답변 유도
     6 # ROS2 표준 센서 메시지 임포트
     7 from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
     8 from std_msgs.msg import Header
     9 # PointCloud2 처리를 위한 모듈 (필요시 sensor_msgs_py 패키지 사용)
    10 # from sensor_msgs_py import point_cloud2
    11
    12 class SensorIntegrationNode(Node):
    13     def __init__(self):
    14         super().__init__('sensor_integration_node')
    15
    16         # 1. 퍼블리셔(Publisher) 생성
    17         # LiDAR (2D)
    18         self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
    19         # mmWave Radar (Point Cloud 형태로 가정)
    20         self.mmwave_pub = self.create_publisher(PointCloud2, '/radar/mmwave', 10)
    21         # 일반 Radar (자율주행용, Point Cloud 형태로 가정)
    22         self.auto_radar_pub = self.create_publisher(PointCloud2, '/radar/front', 10)
    23         # 일반 카메라
    24         self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
    25         # 열화상 카메라
    26         self.thermal_pub = self.create_publisher(Image, '/thermal/image_raw', 10)
    27         # IMU
    28         self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
    29
    30         # 2. 타이머 설정 (예: 10Hz로 데이터 퍼블리시)
    31         timer_period = 0.1  # seconds
    32         self.timer = self.create_timer(timer_period, self.timer_callback)
    33         
    34         self.get_logger().info('Sensor Integration Node has been started.')
    35
    36     def timer_callback(self):
    37         """주기적으로 모든 센서 데이터를 수집하고 퍼블리시하는 메인 루프"""
    38         now = self.get_clock().now().to_msg()
    39
    40         self.publish_lidar(now)
    41         self.publish_mmwave_radar(now)
    42         self.publish_auto_radar(now)
    43         self.publish_camera(now)
    44         self.publish_thermal_camera(now)
    45         self.publish_imu(now)
    46
    47     def publish_lidar(self, timestamp):
    48         """LiDAR 하드웨어에서 데이터를 읽어와서 발행"""
    49         msg = LaserScan()
    50         msg.header = Header(stamp=timestamp, frame_id='laser_link')
    51         msg.angle_min = -math.pi
    52         msg.angle_max = math.pi
    53         msg.angle_increment = math.pi / 180.0  # 1도 단위
    54         msg.time_increment = 0.0
    55         msg.scan_time = 0.1
    56         msg.range_min = 0.12
    57         msg.range_max = 20.0
    58         
    59         # TODO: 실제 LiDAR SDK/API를 호출하여 msg.ranges 배열 채우기
    60         msg.ranges = [5.0] * 360  # 임시 데이터 (모든 방향 5m 거리에 장애물)
    61         msg.intensities = [100.0] * 360
    62         
    63         self.lidar_pub.publish(msg)
    64
    65     def publish_mmwave_radar(self, timestamp):
    66         """mmWave 하드웨어 통신 코드 연동"""
    67         msg = PointCloud2()
    68         msg.header = Header(stamp=timestamp, frame_id='mmwave_link')
    69         # TODO: mmWave 레이더 데이터 파싱 로직 구현 (point_cloud2.create_cloud 사용 권장)
    70         self.mmwave_pub.publish(msg)
    71
    72     def publish_auto_radar(self, timestamp):
    73         """차량용 Radar(자율주행용) 통신 코드 연동"""
    74         msg = PointCloud2()
    75         msg.header = Header(stamp=timestamp, frame_id='auto_radar_link')
    76         # TODO: CAN 통신이나 이더넷을 통해 받은 Radar 타겟 데이터를 PointCloud2로 변환
    77         self.auto_radar_pub.publish(msg)
    78
    79     def publish_camera(self, timestamp):
    80         """USB/IP 카메라 영상 데이터 연동"""
    81         msg = Image()
    82         msg.header = Header(stamp=timestamp, frame_id='camera_link')
    83         # TODO: OpenCV(cv2.VideoCapture) 등을 통해 프레임을 획득하고 CvBridge로 변환
    84         # 예: msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    85         self.camera_pub.publish(msg)
    86
    87     def publish_thermal_camera(self, timestamp):
    88         """열화상 카메라(FLIR 등) 영상 데이터 연동"""
    89         msg = Image()
    90         msg.header = Header(stamp=timestamp, frame_id='thermal_link')
    91         # TODO: 열화상 카메라 SDK 연동. 흑백 혹은 온도값 배열이므로 encoding 설정 주의
    92         # 예: msg = self.cv_bridge.cv2_to_imgmsg(thermal_image, encoding="mono8" 또는 "mono16")
    93         self.thermal_pub.publish(msg)
    94
    95     def publish_imu(self, timestamp):
    96         """IMU (가속도, 각속도, 지자기 등) 센서 연동"""
    97         msg = Imu()
    98         msg.header = Header(stamp=timestamp, frame_id='imu_link')
    99         
   100         # TODO: IMU 시리얼/I2C 통신 코드 연결하여 실제 센서 값 대입
   101         msg.orientation.x = 0.0
   102         msg.orientation.y = 0.0
   103         msg.orientation.z = 0.0
   104         msg.orientation.w = 1.0  # 단위 쿼터니언 (회전 없음)
   105         
   106         msg.angular_velocity.x = 0.0
   107         msg.angular_velocity.y = 0.0
   108         msg.angular_velocity.z = 0.0
   109         
   110         msg.linear_acceleration.x = 0.0
   111         msg.linear_acceleration.y = 0.0
   112         msg.linear_acceleration.z = 9.81  # 중력 가속도
   113
   114         self.imu_pub.publish(msg)
   115
   116 def main(args=None):
   117     rclpy.init(args=args)
   118     node = SensorIntegrationNode()
   119     try:
   120         rclpy.spin(node)
   121     except KeyboardInterrupt:
   122         pass
   123     finally:
   124         node.destroy_node()
   125         rclpy.shutdown()
   126
   127 if __name__ == '__main__':
   128     main()