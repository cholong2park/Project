from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # construction 패키지의 공유 디렉토리 가져오기
    pkg_construction = get_package_share_directory('construction')

    # SDF 파일 경로
    world_path = os.path.join(pkg_construction, 'worlds', 'Construction_world.sdf')

    # LaunchDescription 생성
    return LaunchDescription([
        # Gazebo 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_path}.items(),
        )
    ])
