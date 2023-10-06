from ament_index_python.packages import get_package_share_directory

from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = "edubot"
    ld = LaunchDescription()

    start_sim = Node(package = pkg_name,
                     name = "robot_sim",
                     executable = "robot_sim")

    tf_pub = Node(package = pkg_name,
                  name = "tf_publisher",
                  executable = "tf_publisher")
    
    
    ld.add_action(start_sim)
    ld.add_action(tf_pub)

    return ld