# =============================================================================
#                         ROS2 launch path utils methods
#
# (c) DTU Electro 2025
# Author: Geoffrey Côte
#
# Part of the raubase_core_python package
# This python file gather several methods useful to write path in launch files.
# =============================================================================

from enum import Enum
import sys
import os
from typing import Text

from ament_index_python import get_package_share_directory
from launch import LaunchContext, Substitution
from launch.substitutions import TextSubstitution


# =============================================================================
#   PATHS
# =============================================================================
class PathUtils:
    """
    Collection of path util methods for launch files.

    Author: Geoffrey Côte
    (c) DTU Electro - 2025
    """

    class PathClass(Enum):
        NONE = -1
        ABSOLUTE_PATH = 0
        RELATIVE_PATH = 1
        PACKAGE_PATH = 2

    @staticmethod
    def _determine_path_class(path: str) -> PathClass:
        """
        Determine the path class with simple conditions

        Args:
            path (str): the path to determine the class

        Returns:
            PathClass: the class of this path
        """
        if path[0] == "/":
            return PathUtils.PathClass.ABSOLUTE_PATH
        if "::" in path:
            return PathUtils.PathClass.PACKAGE_PATH
        return PathUtils.PathClass.RELATIVE_PATH

    @staticmethod
    def package2realpath(path: str) -> str:
        """
        Return the real path assiociated with the given package path with
        the format:
            my_package::share_subdirectory/.../object.ext

        Args:
            path (str): the package path

        Returns:
            str: the real path of the package
        """
        pkg_split = path.split("::")
        if len(pkg_split) < 2:
            raise RuntimeError(
                f'Malformated package path, expected pkg::path but got less: "{path}"'
            )

        # Get share path
        share_dir = get_package_share_directory(pkg_split[0])
        return os.path.join(share_dir, *pkg_split[1].split("/"))

    @staticmethod
    def get_object_path(path: str) -> str:
        """
        This method parse and returns the given object path.
        It accepts several path format:
            - absolute path: /home/.../object.ext
            - package path: my_package::my_inner_path_in_share/.../object.ext
                or my_package::myobject.ext

        Args:
            path (str): _description_

        Returns:
            str: _description_
        """

        # Get class
        match PathUtils._determine_path_class(path):
            case PathUtils.PathClass.ABSOLUTE_PATH:
                return path
            case PathUtils.PathClass.RELATIVE_PATH:
                return os.path.realpath(path)
            case PathUtils.PathClass.PACKAGE_PATH:
                return PathUtils.package2realpath(path)
        return ""


# =============================================================================
#   Substitutions
# =============================================================================
class AdvPathSubstitution(Substitution):
    """
    Substitution that perform an advanced path lookup.
    It accepts:
        - absolute path: /root_dir/.../object.ext
        - relative path: local_dir/.../object.ext
        - package  path: my_pkg::share_subdir/.../object.ext

    Args:
        substitution (Substitution | str): the path to the world file
    """

    def __init__(self, substitution: Substitution | str) -> None:
        if type(substitution) is str:
            self._substitution = TextSubstitution(text=substitution)
        else:
            self._substitution: Substitution = substitution  # type: ignore

    def describe(self) -> Text:
        return "AdvPathSubstitution({})".format(self._substitution.describe())

    def perform(self, context: LaunchContext) -> str:
        return PathUtils.get_object_path(self._substitution.perform(context))
