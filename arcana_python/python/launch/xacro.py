# =============================================================================
#                         ROS2 launch xacro utils methods
#
# (c) DTU Electro 2025
# Author: Geoffrey Côte
#
# Part of the raubase_core_python package
# This python file gather several methods useful to write path in launch files.
# =============================================================================
# Package imports
from .path import AdvPathSubstitution
from .__utils import normalize, SubstitionsInput

# Standard python import
import os
import tempfile
import subprocess
from typing import Text

# ROS imports
from rclpy.logging import get_logger
from launch import LaunchContext, LaunchDescription, Substitution
from launch.actions import (
    SetLaunchConfiguration,
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    ExecuteProcess,
)
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.substitutions import LaunchConfiguration


# =============================================================================
#   XACRO
# =============================================================================
def xacro_object(object_path: str) -> str:
    """
    Run xacro on the given file and export it in a temporary file.

    Args:
        object (str): the path to the object file to run xacro on

    Returns:
        str: the path to the temporary file for the object
    """
    logger = get_logger("LaunchXacro")
    if not os.path.exists(object_path):
        raise RuntimeError('File "{}" does not exist!'.format(object_path))

    file_name, extension = object_path.split(os.sep)[-1].split(".")

    # Make temporary file
    fd, name = tempfile.mkstemp(prefix=file_name, suffix="." + extension)
    os.chmod(name, 0o744)
    logger.info(
        f'Creating temporary file "{name}" for the output of object "{file_name}"'
    )
    os.close(fd)

    # Running xacro
    try:
        out = subprocess.run(
            ["xacro", "-o", name, object_path],
            check=False,
            capture_output=True,
        )
        out.check_returncode()
    except subprocess.CalledProcessError as e:
        logger.info(
            "Xacro failed with code {}\n--- STDOUT ---\n{}\n--- STDERR ---\n{}".format(
                e.returncode,
                bytes.decode(e.stdout),
                bytes.decode(e.stderr),
            )
        )
        raise RuntimeError("XACRO error")

    return name


def setup_xacro_object(
    ld: LaunchDescription,
    conf_name: str,
    def_value: SubstitionsInput | None = None,
) -> None:
    """
    Setup the launch description for running xacro on the given configuration
    name. What this function do is:

    - Declare launch argument <conf_name> for user to set it up

    - Add LaunchConfiguration <conf_name>_file with full file path

    Args:
        ld (LaunchDescription): the launch description to fill
        conf_name (str): the configuration name
    """
    ld.add_action(
        DeclareLaunchArgument(  # Declare launch argument with the object file name
            conf_name,
            description=f"File path for object `{conf_name}`",
            default_value=def_value,
        )
    )
    ld.add_action(
        SetLaunchConfiguration(  # Set launch configuration with file path
            f"{conf_name}_file",
            value=XacroCommand(AdvPathSubstitution(LaunchConfiguration(conf_name))),
        )
    )
    ld.add_action(
        RegisterEventHandler(  # Cleaning up xacro temporary file
            OnShutdown(
                on_shutdown=[
                    LogInfo(
                        msg=[
                            "Cleaning temp xacro file ",
                            LaunchConfiguration(f"{conf_name}_file"),
                        ]
                    ),
                    ExecuteProcess(
                        cmd=["rm", LaunchConfiguration(f"{conf_name}_file")]
                    ),
                ]
            )
        )
    )


# =============================================================================
#   Substitution
# =============================================================================
class XacroCommand(Substitution):
    """
    Substitution to run a xacro command on the given file path

    Args:
        Substitution (_type_): _description_
    """

    def __init__(self, file: Substitution | str) -> None:
        self._file = normalize(file)

    def describe(self) -> Text:
        return "XacroCommand({})".format(self._file.describe())

    def perform(self, context: LaunchContext) -> str:
        return xacro_object(self._file.perform(context))
