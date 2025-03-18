# =============================================================================
#                         ROS2 generator utils methods
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file gather several methods useful to quickly write launch files.
# =============================================================================

import os
from typing import Iterable, Tuple

from ament_index_python import get_package_share_directory
from launch import SomeSubstitutionsType
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# =============================================================================
#   Generators
# =============================================================================
def include_launch_file(
    pkg: str,
    file: str,
    launch_args: (
        Iterable[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]] | None
    ) = None,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), *file.split("/"))
        ),
        launch_arguments=launch_args,
    )
