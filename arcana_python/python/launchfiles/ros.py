# =============================================================================
#                         ROS2 installation utils methods
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file gather several methods useful when you want to get
# informations with the ROS installation
# =============================================================================
from enum import IntEnum, Enum
from subprocess import PIPE, run
from typing import Literal, List


class ROS:

    class Version(IntEnum):
        ARDENT = 0
        CRYSTAL = 1
        DASHING = 2
        ELOQUENT = 3
        FOXY = 4
        GALACTIC = 5
        HUMBLE = 6
        IRON = 7
        JAZZY = 8
        ABOVE = 100

    class Name:
        ARDENT = "ardent"
        CRYSTAL = "crystal"
        DASHING = "dashing"
        ELOQUENT = "eloquent"
        FOXY = "foxy"
        GALACTIC = "galactic"
        HUMBLE = "humble"
        IRON = "iron"
        JAZZY = "jazzy"
        ABOVE = "above"

    @staticmethod
    def from_string(distro: str) -> "ROS.Version":
        match distro:
            case str(ROS.Name.ARDENT):
                return ROS.Version.ARDENT
            case ROS.Name.CRYSTAL:
                return ROS.Version.CRYSTAL
            case ROS.Name.DASHING:
                return ROS.Version.DASHING
            case ROS.Name.ELOQUENT:
                return ROS.Version.ELOQUENT
            case ROS.Name.FOXY:
                return ROS.Version.FOXY
            case ROS.Name.GALACTIC:
                return ROS.Version.GALACTIC
            case ROS.Name.HUMBLE:
                return ROS.Version.HUMBLE
            case ROS.Name.IRON:
                return ROS.Version.IRON
            case ROS.Name.JAZZY:
                return ROS.Version.JAZZY
            case _:
                return ROS.Version.ABOVE

    @staticmethod
    def to_string(version: "ROS.Version") -> str:
        match version:
            case ROS.Version.ARDENT:
                return ROS.Name.ARDENT
            case ROS.Version.CRYSTAL:
                return ROS.Name.CRYSTAL
            case ROS.Version.DASHING:
                return ROS.Name.DASHING
            case ROS.Version.ELOQUENT:
                return ROS.Name.ELOQUENT
            case ROS.Version.FOXY:
                return ROS.Name.FOXY
            case ROS.Version.GALACTIC:
                return ROS.Name.GALACTIC
            case ROS.Version.HUMBLE:
                return ROS.Name.HUMBLE
            case ROS.Version.IRON:
                return ROS.Name.IRON
            case ROS.Version.JAZZY:
                return ROS.Name.JAZZY
            case _:
                return ROS.Name.ABOVE

    @staticmethod
    def get_version() -> "ROS.Version":
        """
        Fetch the loaded ROS version in the path

        Returns:
            ROSVersion: the actual ros version
        """
        proc = run(["rosversion", "-d", "-s"], stdout=PIPE)
        return ROS.from_string(proc.stdout.decode())

    @staticmethod
    def more_recent_than(v: "ROS.Version", equal: bool = True) -> bool:
        """
        Test whether the current ros installation is more recent than
        the one given as argument.

        Args:
            v (ROS.Version): the version to compare to
            equal (bool, optional): should it return True on equality

        Returns:
            bool: True if the installation is more recent (or equal) then
            the version in argument
        """
        return ROS.get_version() >= v if equal else ROS.get_version() > v

    @staticmethod
    def older_than(v: "ROS.Version", equal: bool = True) -> bool:
        """
        Test whether the current ROS installation is older than the
        one given as argument.

        Args:
            v (ROS.Version): the version to compare to
            equal (bool, optional): should it return True on equality

        Returns:
            bool: True if the installation is older (or equal) then
            the version in argument
        """
        return not ROS.more_recent_than(v, not equal)
