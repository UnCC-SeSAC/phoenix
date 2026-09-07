#!/usr/bin/env python3
"""PHM 감시 노드만 띄웁니다. 주행 스택은 이미 떠 있다고 가정합니다.

무엇이 필요한가
---------------
이 노드는 아래 토픽을 **구독만** 합니다. 하나라도 없으면 그 축은 판정을 멈추고
`/phm/status` 의 `blocked_reason` 에 이유가 적힙니다.

    지령  /controller/cmd_vel 또는 /cmd_vel      (둘 다 구독하고 오는 쪽을 씁니다)
    요레이트 /ros_robot_controller/imu_raw
    전진속도 /odom_rf2o        <- rf2o_laser_odometry 가 떠 있어야 합니다
    배터리  /ros_robot_controller/battery

★★ rf2o 출력을 EKF 와 분리합니다 (`rf2o_topic` 기본 /phm/odom_rf2o)
------------------------------------------------------------------
이 브랜치의 `ekf.yaml` 은 **`odom1: odom_rf2o`** 입니다. 즉 EKF 가 rf2o 원본을
그대로 융합하도록 설정돼 있습니다. 그런데 원본은 라이다가 180도 돌아 장착돼 있어
x,y 부호가 반대이고 covariance 가 전부 0 입니다.

`albitro/data_dashboard` 에서는 문제가 없었습니다 — 거기 `ekf.yaml` 은
`odom_rf2o_fixed`(릴레이 출력)를 보므로 원본이 비어 있어도 됐습니다. **여기서는
릴레이와 ekf.yaml 을 안 가져왔기 때문에 원본 토픽이 곧 EKF 입력입니다.**

그래서 이 런치가 `/odom_rf2o` 로 발행하면 **EKF 가 부호 반대인 위치를 먹기 시작합니다**
(실측: odom_raw 와 이동 벡터 148~159도 차이). 주행 스택과 같이 띄우면 조용히
로컬라이제이션이 망가집니다.

PHM 은 토픽 이름과 무관하게 동작하므로, rf2o 출력을 **`/phm/odom_rf2o` 로 remap** 해서
EKF 의 시야 밖에 둡니다. EKF 는 종전대로 odom0(휠) + imu0 으로만 돕니다 — 이 런치를
띄우기 전과 **완전히 같은 거동**입니다.

EKF 에도 rf2o 를 먹이려면 릴레이와 ekf.yaml 을 함께 이식한 뒤
`rf2o_topic:=/odom_rf2o` 로 되돌리세요. 그 전에는 절대 그러지 마세요.

★ 이 브랜치(albitro/phm_feat)에는 covariance 릴레이가 없습니다
--------------------------------------------------------------
릴레이(`controller` 패키지의 `rf2o_covariance_relay`)와 그 짝인 `ekf.yaml` 변경은
`albitro/data_dashboard` 에만 있고 여기로 가져오지 않았습니다. 그것들은 EKF 의
위치 추정을 바꾸는 변경이라 주행 거동에 영향을 주고, PHM 과는 성격이 다릅니다.

**PHM 은 릴레이 없이 완전히 동작합니다** — 원본 `/odom_rf2o` 를 읽고 부호를 스스로
보정하기 때문입니다. 그래서 `with_relay` 기본값이 **false** 입니다. 켜면 그 패키지에
entry point 가 없어 런치 전체가 실패합니다.

★ rf2o 는 이 런치가 직접 띄웁니다 (`with_rf2o` 기본 true)
--------------------------------------------------------
이 브랜치에서도 **아무도 rf2o 를 안 띄웁니다.**
`controller/launch/rf2o_laser_odometry.launch.py` 파일은 있지만 include 하는 곳이
없습니다(`albitro/phm_collect` 의 controller.launch.py:60 에만 있습니다). 벤더 스택
(`/home/ubuntu/ros2_ws`)도 마찬가지입니다.

그래서 PHM 이 자기 의존을 스스로 챙깁니다. 주행 스택(`controller.launch.py`)은
건드리지 않습니다 — 다른 사람이 쓰는 브랜치이고, 거기에 넣으면 rf2o CPU 가 항상
붙습니다.

이미 다른 데서 rf2o 를 띄우고 있다면 `with_rf2o:=false` 로 끄세요. 두 번 띄우면
/odom_rf2o 발행자가 둘이 되어 스캔매칭 결과가 섞입니다.

두 토픽의 차이 (헷갈리기 쉽습니다)
----------------------------------
    /odom_rf2o          rf2o 원본. 라이다가 180도 돌아 장착돼 있어 x,y 와 vx,vy 의
                        부호가 반대입니다(실측: 전진 지령 중 96% 가 음수, 중앙 -0.188).
                        covariance 36칸이 전부 0 입니다.
    /odom_rf2o_fixed    rf2o_covariance_relay 가 부호를 뒤집고 covariance 를 채운 것.
                        **EKF 는 반드시 이쪽**을 써야 합니다(ekf.yaml odom1).

PHM 노드는 **원본**을 읽고 부호를 스스로 보정합니다
(`phm_detect_core.AXES['fwd']['sign'] = -1.0`). twist.linear.x 하나만 쓰고
covariance 는 안 보기 때문입니다. 같은 보정이 두 곳에 있는 것이 맞습니다 —
'중복' 이라고 한쪽을 지우면 다른 쪽이 조용히 틀려집니다.

    ros2 launch phm_collect phm_monitor.launch.py
    ros2 launch phm_collect phm_monitor.launch.py with_rf2o:=false   # 이미 떠 있을 때
    ros2 launch phm_collect phm_monitor.launch.py with_relay:=true   # 릴레이를 이식한 뒤에만
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    with_rf2o = LaunchConfiguration('with_rf2o')
    with_relay = LaunchConfiguration('with_relay')
    rf2o_launch = os.path.join(
        get_package_share_directory('controller'), 'launch',
        'rf2o_laser_odometry.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_rf2o', default_value='true',
            description='rf2o + covariance 릴레이를 같이 띄웁니다. 이 브랜치에서는 '
                        '아무도 rf2o 를 안 띄우므로 기본이 true 입니다. 이미 떠 '
                        '있으면 false 로 두세요 — 두 번 띄우면 발행자가 둘이 됩니다.'),
        DeclareLaunchArgument(
            'with_relay', default_value='false',
            description='rf2o covariance 릴레이를 같이 띄웁니다. 이 브랜치에는 '
                        'controller 패키지에 rf2o_covariance_relay entry point 가 '
                        '없으므로 기본이 false 입니다 — true 로 두면 런치가 실패합니다. '
                        'PHM 은 원본 /odom_rf2o 를 읽으므로 릴레이가 없어도 됩니다.'),
        DeclareLaunchArgument(
            'rf2o_topic', default_value='/phm/odom_rf2o',
            description='rf2o 가 발행할 토픽이자 PHM 이 구독할 토픽. 기본값이 '
                        '/odom_rf2o 가 아닌 이유는 위 독스트링 참고 — 이 브랜치의 '
                        'EKF 가 /odom_rf2o 를 융합하도록 설정돼 있어서, 그대로 쓰면 '
                        '부호가 반대인 위치를 EKF 가 먹습니다.'),
        DeclareLaunchArgument('status_topic', default_value='/phm/status'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        # rf2o 는 odom_topic 파라미터로 /odom_rf2o 에 발행합니다. 그 이름을
        # 여기서 갈아끼워 EKF 와 겹치지 않게 합니다(독스트링 ★★ 참고).
        GroupAction(
            condition=IfCondition(with_rf2o),
            actions=[
                SetRemap(src='/odom_rf2o', dst=LaunchConfiguration('rf2o_topic')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rf2o_launch)),
            ]),
        # ★ 릴레이는 이 브랜치에 없습니다 (with_relay 기본 false).
        # 릴레이는 EKF 의 odom1(= odom_rf2o_fixed) 입력을 만드는 쪽이고,
        # ekf.yaml 변경과 짝입니다. 둘 다 여기로 가져오지 않았습니다.
        # PHM 자체는 원본 /odom_rf2o 를 읽고 부호를 스스로 보정하므로 영향이 없습니다.
        Node(
            package='controller',
            executable='rf2o_covariance_relay',
            name='rf2o_covariance_relay',
            output='screen',
            condition=IfCondition(with_relay)),
        Node(
            package='phm_collect',
            executable='phm_monitor',
            name='phm_monitor',
            output='screen',
            parameters=[{
                'status_topic': LaunchConfiguration('status_topic'),
                # 위에서 remap 한 이름과 반드시 같아야 합니다.
                'rf2o_topic': LaunchConfiguration('rf2o_topic'),
                # 노드가 double 로 선언한 파라미터입니다. LaunchConfiguration 은
                # 문자열이라 그대로 넘기면 기동 시 타입 오류가 납니다
                # (firefighter_ui.launch.py 가 ParameterValue 를 쓰는 것과 같은 이유).
                'publish_period_sec': ParameterValue(
                    LaunchConfiguration('publish_period_sec'), value_type=float),
            }],
        ),
    ])
