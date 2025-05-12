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
from subprocess import PIPE, run
from ..ros.distro import ROSD


class ROS:

    @staticmethod
    def get_version() -> ROSD.Version:
        """
        Fetch the loaded ROS version in the path

        Returns:
            ROSVersion: the actual ros version
        """
        proc = run(["rosversion", "-d", "-s"], stdout=PIPE)
        return ROSD.from_string(proc.stdout.decode())

    @staticmethod
    def more_recent_than(v: ROSD.Version, equal: bool = True) -> bool:
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
    def older_than(v: ROSD.Version, equal: bool = True) -> bool:
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
