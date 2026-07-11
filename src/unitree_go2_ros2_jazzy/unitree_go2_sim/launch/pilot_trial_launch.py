import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    trial_name = LaunchConfiguration("trial_name")

    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    explore_params_file = os.path.join(unitree_go2_sim, "config", "explore_params.yaml")

    # Adjust this if coverage_logger.py lives somewhere else -- e.g. drop it
    # into unitree_go2_sim/launch/ and update this path to match.
    coverage_logger_script = os.path.join(
        unitree_go2_sim, "launch", "coverage_logger.py"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_trial_name = DeclareLaunchArgument(
        "trial_name",
        default_value="trial_1",
        description="Output CSV name for this trial (e.g. trial_1, trial_2, ...)."
        " Pass a new value each run: ros2 launch ... trial_name:=trial_2",
    )

    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[
            explore_params_file,
            # {"use_sim_time": use_sim_time},
        ],
    )

    coverage_logger = ExecuteProcess(
        cmd=[
            "python3", coverage_logger_script,
            "--ros-args",
            "-p", ["output_csv:=/home/ros2_ws/src/", trial_name, ".csv"],
            "-p", ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    )

    # Primary shutdown trigger: watch explore_node's own stdout for the
    # message it prints when no frontiers remain. Confirmed log text for
    # this exact package (robo-friends/m-explore-ros2 explore_lite).
    def on_explore_output(event):
        text = event.text.decode(errors="replace")
        if "All frontiers traversed" in text or "stopping" in text.lower():
            return [
                LogInfo(msg=">>> explore_lite reports finished -- shutting down trial <<<"),
                EmitEvent(event=Shutdown(reason="exploration complete")),
            ]
        return None

    stop_on_completion = RegisterEventHandler(
        OnProcessIO(
            target_action=explore_node,
            on_stdout=on_explore_output,
            on_stderr=on_explore_output,
        )
    )

    # Safety net: if the completion text never appears (e.g. it changes in a
    # future version of the package, or exploration genuinely never
    # terminates), force-stop after 5 minutes so a trial can't hang forever
    # and silently ruin your pilot run.
    timeout_shutdown = TimerAction(
        period=300.0,
        actions=[
            LogInfo(msg=">>> 5 minute trial timeout reached -- shutting down <<<"),
            EmitEvent(event=Shutdown(reason="trial timeout")),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_trial_name,
            explore_node,
            coverage_logger,
            stop_on_completion,
            timeout_shutdown,
        ]
    )
