from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="armmodel", package_name="armmodel_moveit_config")
        .robot_description(file_path="config/armmodel.urdf.xacro")  # 改成你真实的 xacro 路径
        .robot_description_semantic(file_path="config/armmodel.srdf")  # SRDF
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # 你的控制器配置
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
