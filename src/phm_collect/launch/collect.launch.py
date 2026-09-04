#!/usr/bin/env python3
"""PHM 잔차 수집 전용 최소 기동.

무엇을 띄우나
-------------
잔차 계산에 실제로 필요한 것만 띄웁니다.

    지령  : /controller/cmd_vel (조이스틱)  ->  /ros_robot_controller/set_motor
    실측  : /ros_robot_controller/imu_raw (자이로), /odom_rf2o (스캔매칭)
    참고  : /imu, /odom_raw, /odom, /scan_raw, /ros_robot_controller/battery

bringup.launch.py 가 띄우는 것 중 아래는 **띄우지 않습니다** — 잔차와 무관하고
CPU 만 먹습니다: ascamera(depth camera), start_app 의 5개 앱 노드, rosbridge_websocket,
web_video_server, startup_check, init_pose, 그리고 nav2/slam/yolo/mission 계층 전부.

왜 기존 런치를 include 하지 않나
--------------------------------
로봇 런타임은 ``need_compile=False`` 입니다. 그러면 controller.launch.py:38 과
odom_publisher.launch.py:31-33 이 패키지 경로를 ``/home/ubuntu/ros2_ws/src/...``
(벤더 워크스페이스)로 잡습니다. phoenix 의 런치가 실행돼도 그 안에서 include 하는
하위 런치와 ekf.yaml 은 **벤더 사본**을 읽어서, phoenix 쪽 수정이 조용히 무시됩니다.

그래서 여기서는 기존 런치 체인을 일절 include 하지 않고 필요한 노드를 직접 선언합니다.
경로는 전부 get_package_share_directory 로 풀기 때문에 need_compile 에 영향받지 않습니다.

사용법
------
    # 조이스틱 수동 주행 + rf2o + EKF (기본)
    ros2 launch phm_collect collect.launch.py

    # EKF 없이 최소 구성 (CPU 최소)
    ros2 launch phm_collect collect.launch.py use_ekf:=false

    # nav2 등 외부에서 /cmd_vel 을 쏠 때 (조이스틱 끔)
    ros2 launch phm_collect collect.launch.py use_joy:=false

주의
----
* ``max_linear``/``max_angular`` 기본값은 0.2 / 0.5 입니다. 이건 ``/cmd_vel`` 경로가
  odom_publisher_node.py:181-192 에서 클램프하는 값과 같습니다. 조이스틱이 쓰는
  ``/controller/cmd_vel`` 경로에는 클램프가 없으므로(:132 -> cmd_vel_callback 직결)
  여기서 맞춰주지 않으면 nav 이 절대 못 내는 속도까지 나가서, 그 데이터로 잡은
  임계값이 실제 주행에 안 맞게 됩니다.
* 이 로봇은 좌우쌍 차동(skid-steer)인데 MACHINE_TYPE 은 MentorPi_Mecanum 입니다.
  mecanum 기구학에 linear.y=0 을 넣으면 차동이 나오기 때문입니다. 즉 조이스틱
  **왼쪽 스틱 좌우(lx = linear.y)는 건드리지 마세요** — 하드웨어가 못 내는 지령입니다.
  전진후진은 왼쪽 스틱 상하(ly), 회전은 오른쪽 스틱 좌우(rx) 입니다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    controller_share = get_package_share_directory('controller')
    description_share = get_package_share_directory('mentorpi_description')
    calibration_share = get_package_share_directory('calibration')

    urdf_path = os.path.join(description_share, 'urdf', 'mentorpi.xacro')
    calib_params = os.path.join(controller_share, 'config', 'calibrate_params.yaml')
    imu_calib_file = os.path.join(calibration_share, 'config', 'imu_calib.yaml')
    ekf_params = os.path.join(
        get_package_share_directory('phm_collect'), 'config', 'ekf_collect.yaml')

    # ---- 인자 ----
    args = [
        DeclareLaunchArgument('machine_type', default_value='MentorPi_Mecanum',
                              description='odom_publisher / joystick_control 이 읽는 환경변수'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_rf2o', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('use_joy', default_value='true'),
        DeclareLaunchArgument('use_imu_filter', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='scan_raw'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_frame',
                              description='실측한 /scan_raw 의 header.frame_id 와 같아야 rf2o 의 TF 조회가 됩니다'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('rf2o_freq', default_value='10.0'),
        DeclareLaunchArgument('max_linear', default_value='0.2',
                              description='/cmd_vel 클램프(±0.2)와 동일하게 맞춘 값'),
        DeclareLaunchArgument('max_angular', default_value='0.5',
                              description='/cmd_vel 클램프(±0.5)와 동일하게 맞춘 값'),
    ]

    machine_type = LaunchConfiguration('machine_type')
    scan_topic = LaunchConfiguration('scan_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')

    # odom_publisher_node.py:96 과 joystick_control.py:38 이 os.environ 으로 직접 읽습니다.
    # 없으면 KeyError 로 죽고, 틀리면 cmd_vel_callback 에 해당 분기가 없어 모터로
    # 아무것도 안 나갑니다(§11 의 '차동 지령이 안 나감'과 같은 증상).
    set_machine_type = SetEnvironmentVariable('MACHINE_TYPE', machine_type)

    # ---- TF ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
            'use_sim_time': False,
        }],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/controller_manager/joint_states'], 'rate': 20.0}],
    )

    # ---- 하드웨어: /ros_robot_controller/imu_raw, /battery, set_motor 구독 ----
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
        parameters=[{'imu_frame': imu_frame}],
    )

    # ---- 구동: cmd_vel -> set_motor, /odom_raw 발행 ----
    odom_publisher_node = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[calib_params, {
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'pub_odom_topic': True,
            'machine_type': machine_type,
        }],
    )

    # ---- IMU 보정 + 필터 -> /imu ----
    # ros_robot_controller 가 시리얼을 열고 imu_raw 를 내기 시작해야 의미가 있어서
    # 벤더 런치와 동일하게 5초 늦춥니다.
    imu_group = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='imu_calib',
                executable='apply_calib',
                name='imu_calib',
                output='screen',
                parameters=[{'calib_file': imu_calib_file}],
                remappings=[('raw', '/ros_robot_controller/imu_raw'),
                            ('corrected', 'imu_corrected')],
            ),
            Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='imu_filter',
                output='screen',
                parameters=[{'use_mag': False, 'do_bias_estimation': True,
                             'do_adaptive_gain': True, 'publish_debug_topics': False}],
                remappings=[('/tf', 'tf'), ('/imu/data_raw', 'imu_corrected'),
                            ('imu/data', 'imu')],
            ),
        ],
    )

    # ---- 라이다 -> /scan_raw ----
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[{
            'topic_name': 'scan',
            'product_name': 'LDLiDAR_LD19',
            'port_baudrate': 230400,
            'port_name': '/dev/ldlidar',
            'frame_id': lidar_frame,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
        }],
        remappings=[('scan', scan_topic)],
    )

    # ---- rf2o -> /odom_rf2o ----
    # publish_tf 는 반드시 False. EKF 가 publish_tf:true 라 여기서도 켜면
    # odom -> base_footprint TF 를 둘이 동시에 쏴서 TF 트리가 깨집니다.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': scan_topic,
            'odom_topic': 'odom_rf2o',
            'publish_tf': False,
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'init_pose_from_topic': '',
            'freq': LaunchConfiguration('rf2o_freq'),
        }],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # ---- rf2o covariance 릴레이 -> /odom_rf2o_fixed ----
    # rf2o 는 covariance 36칸을 전부 0 으로 채워 발행합니다(live09 실측 확인).
    rf2o_relay_node = Node(
        package='controller',
        executable='rf2o_covariance_relay',
        name='rf2o_covariance_relay',
        output='screen',
    )

    # ---- EKF -> /odom ----
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('odometry/filtered', 'odom')],
    )

    # ---- 조이스틱 -> /controller/cmd_vel ----
    joy_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_joy')),
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{'dev': '/dev/input/js0', 'autorepeat_rate': 20.0}],
            ),
            Node(
                package='peripherals',
                executable='joystick_control',
                name='joystick_control',
                output='screen',
                parameters=[{
                    'max_linear': LaunchConfiguration('max_linear'),
                    'max_angular': LaunchConfiguration('max_angular'),
                    'disable_servo_control': True,
                }],
            ),
        ],
    )

    return LaunchDescription(args + [
        set_machine_type,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ros_robot_controller_node,
        odom_publisher_node,
        GroupAction(condition=IfCondition(LaunchConfiguration('use_imu_filter')),
                    actions=[imu_group]),
        GroupAction(condition=IfCondition(LaunchConfiguration('use_lidar')),
                    actions=[lidar_node]),
        GroupAction(condition=IfCondition(LaunchConfiguration('use_rf2o')),
                    actions=[rf2o_node]),
        GroupAction(condition=IfCondition(LaunchConfiguration('use_ekf')),
                    actions=[rf2o_relay_node, ekf_node]),
        joy_group,
    ])
