import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    # パス設定

    my_nav_dir = os.path.expanduser('~/ros2_ws/src/my_robot_nav')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')



    # 引数の設定

    map_yaml_file = LaunchConfiguration('map')

    params_file = LaunchConfiguration('params_file')



    declare_map_yaml_cmd = DeclareLaunchArgument(

        'map',

        default_value=os.path.expanduser('~/map_practice/7_14/my_map.yaml'),

        description='Full path to map yaml file to load')



    declare_params_file_cmd = DeclareLaunchArgument(

        'params_file',

        default_value=os.path.join(my_nav_dir, 'config', 'nav2_params.yaml'),

        description='Full path to the ROS2 parameters file to use for all launched nodes')



    # 1. Nav2 本体の起動 (AMCLを除外)

    # navigation_launch.py を利用して主要ノードを起動します

    bringup_cmd = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),

        launch_arguments={

            'use_sim_time': 'False',

            'params_file': params_file,

            'autostart': 'True'

        }.items())



    # 2. Map Server の起動

    map_server_cmd = Node(

        package='nav2_map_server',

        executable='map_server',

        name='map_server',

        output='screen',

        parameters=[{'use_sim_time': False},

                    {'yaml_filename': map_yaml_file}])



    # 3. Lifecycle Manager (Map Server用)

    lifecycle_manager_cmd = Node(

        package='nav2_lifecycle_manager',

        executable='lifecycle_manager',

        name='lifecycle_manager_map',

        output='screen',

        parameters=[{'use_sim_time': False},

                    {'autostart': True},

                    {'node_names': ['map_server']}])



    # map_server + そのlifecycle_managerがマップを読み込み・publishし終える前に
    # Nav2本体(costmap含む)が起動すると、"Can't update static costmap layer,
    # no map received" という警告(競合状態)が繰り返し発生する。
    # 起動順序を保証するため、Nav2本体の起動を数秒遅らせる。
    delayed_bringup_cmd = TimerAction(period=3.0, actions=[bringup_cmd])

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(declare_params_file_cmd)

    ld.add_action(map_server_cmd)

    ld.add_action(lifecycle_manager_cmd)

    ld.add_action(delayed_bringup_cmd)



    return ld 


