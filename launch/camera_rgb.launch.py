from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_rgb",
            node_executable="camera_rgb",
            node_name="camera",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"read_video": False},
                {"video_path": ""}  # absolute path of *.mp4 file. Example: /home/<user>/Desktop/sample.mp4
            ]
        )
    ])
