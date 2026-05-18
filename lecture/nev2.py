import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml

# launch.actions / launch_ros.actions 계열 : 실행할 노드를 “파이썬 코드로” 조립해 `LaunchDescription` 으로 반환
# RewrittenYaml / ParameterFile: 사용자가 넘긴 YAML 파라미터를 런치 시점에 가공해서 노드에 주입



def generate_launch_description(): # 런치 인자 입력하여 지정된 인자를 반환
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Variables (입력 변수들, 런치인자와 다른 개념)
    lifecycle_nodes = ['map_saver']
    # SLAM을 통해 만든 지도를 맵 저장 서버(map_saver_server)에 올리기 위한 변수
    # 이후 라이프사이클 매니저가 lifecycle_nodes의 리스트(['map_saver'])를 넘겨 받음
    bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') # nav2_bringup과 slam_toolbox의 설치 경로를 반환
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')
    # 런치 파일 경로를 launch/online_sync_launch.py 로 설정

    # Create our own temporary YAML files that include substitutions
    param_substitutions = { # YAML의 use_sim_time 값을 런치 인자로 덮어쓰도록 선언
        'use_sim_time': use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml( # params_file 을 런치 시점에 가공하는 단계
            source_file=params_file, #  주입 가능한 파라미터 세트 객체
            root_key=namespace, # 네임스페이스가 비어있으면 최상단에, 지정하면 해당 키 아래에 파라미터 트리를 배치
            param_rewrites=param_substitutions,
            convert_types=True), # "True" 같은 문자열을 자동으로 실제 불리언(Boolean) 타입 등으로 변환
        allow_substs=True)
    # RewrittenYaml과 allow_substs=True 를 넘기면, 앞선 가공 요소를 반영하여 parameters로 사용
    
    # DeclareLaunchArgument(): 아래 입력 인자들을 정의하여 터미널 명령어나 외부 런치 파일에서 값을 조정할 수 있도록 공개하는 역할
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', # nav2_bringup/params/nav2_params.yaml로 지정해둠으로써, 사용자가 별도의 파라미터 파일을 제공하지 않더라도 로봇이 최소한으로 동작할 수 있게 하는 기본 안전 장치 역할
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    start_map_saver_server_cmd = Node(
        package='nav2_map_server', # 패키지 대상을 지정
        executable='map_saver_server', # 실행 대상을 지정
        output='screen',
        respawn=use_respawn, # 노드 크래시 발생 시 재기동 여부를 런치 인자로 제어
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', log_level], # 로깅 레벨을 외부에서 제어할 수 있도록 설정
        parameters=[configured_params]) # 가공된 YAML 파일의 내용을 맵 세이버 서버의 파라미터로 주입

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager', # 패키지 대상 지정
        executable='lifecycle_manager', # 실행 대상을 지정
        name='lifecycle_manager_slam', # 노드 지정
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}, # 시뮬레이션 타임을 쓸지 여부
                    {'autostart': autostart}, # 런치 후 자동으로 활성화 전이 여부
                    {'node_names': lifecycle_nodes}]) # 관리할 라이프사이클 대상 노드 목록 (관리 대상: lifecycle_nodes=['map_saver'])

    has_slam_toolbox_params = HasNodeParams(source_file=params_file, 
                                            node_name='slam_toolbox')
    # params_file 안에 'slam_toolbox' 섹션이 있는지 검사
        # 잘못된 YAML을 전달하면 slam_toolbox가 잘못된 파라미터로 덮여서 실행에 실패 가능
        # slam_toolbox 섹션 무: 사용자 파일(YAML)은 무시하고 slam_toolbox 기본 파라미터로 실행
        # slam_toolbox 섹션 유: slam_params_file로 명시하여 전달

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items(), # {'use_sim_time': use_sim_time} 를 사용하여 기본 파라미터로 실행
        condition=UnlessCondition(has_slam_toolbox_params)) # 사용자 YAML에 'slam_toolbox' 섹션이 없을 때만 실행

    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time, # {'use_sim_time': use_sim_time, 'slam_params_file': params_file} 를 통해 사용자 정의 파라미터를 정확히 주입
                        'slam_params_file': params_file}.items(),
        condition=IfCondition(has_slam_toolbox_params)) # 사용자 YAML에 'slam_toolbox' 섹션이 있을 때만 실행

    # Running SLAM Toolbox (Only one of them will be run)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_slam_toolbox_cmd_with_params)