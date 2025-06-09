# =============================================================================
#                         ROS2 launch gazebo utils methods
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file gather several methods useful to write gazebo-related
# commands in launch files.
# =============================================================================

from .path import ConcatenatedPathsSubstitution
from .__utils import normalize_list

from launch import LaunchDescription, Substitution
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration

import os
from typing import List


# =============================================================================
#   Gazebo params
# =============================================================================
def setup_custom_gazebo_models(
    ld: LaunchDescription,
    paths: List[Substitution] | Substitution | str | None = None,
):
    """
    This methods setup custom model paths environment for Gazebo.
    What this function do is:

    - Declare a launch argument "gz_models_path" for modification on launch
    - Set the env variable "GZ_SIM_RESOURCE_PATH" with the following values:
        - the ~/.gazebo/models directory
        - the provided "paths" argument
        - the paths provided in the "gz_models_path" launch argument

    Args:
        ld (LaunchDescription): the launch description to use
        paths (optional): a list of paths to include
    """
    ld.add_action(DeclareLaunchArgument("gz_models_path", default_value=""))
    ld.add_action(
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            value=ConcatenatedPathsSubstitution(
                [
                    TextSubstitution(text=f"{os.path.expanduser('~/.gazebo/models/')}"),
                    *normalize_list(paths),
                    LaunchConfiguration("gz_models_path"),
                ]
            ),
        )
    )
