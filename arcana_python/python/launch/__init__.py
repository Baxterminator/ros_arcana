# =============================================================================
#                  Arcana Python launch module entrypoint
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This file is the entry point of the arcana_python.launch module
# =============================================================================

from .path import PathUtils
from .xacro import XacroCommand, setup_xacro_object, xacro_object
from ._gen import include_launch_file
from .gazebo import setup_custom_gazebo_models
from .ros import ROSD, ROS
from .conditions import (
    ANDConditions,
    ORCondition,
    XORCondition,
    NotCondition,
    TernaryValue,
)
from .substitutions import (
    TextConcat,
    AdvPathSubstitution,
    ConcatenatedPathsSubstitution,
)
