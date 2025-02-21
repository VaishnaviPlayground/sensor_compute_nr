import launch
import launch_ros.actions

def generate_launch_description():
    sensor_process = launch_ros.actions.Node(
        package='sensor_compute',
        executable='sensor_node',
        name='sensor_process')
    central_process = launch_ros.actions.Node(
        package='sensor_compute',
        executable='central_node',
        name='central_process')

    return launch.LaunchDescription([
        sensor_process,
        central_process
    ])
