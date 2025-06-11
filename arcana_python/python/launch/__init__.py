# =============================================================================
#                  Arcana Python launch module entrypoint
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This file is the entry point of the arcana_python.launch module
# =============================================================================

# -----------------------------------------------------------------------------
# Utils methods for launching ROS
# -----------------------------------------------------------------------------
from .path import PathUtils
from .xacro import XacroCommand, setup_xacro_object, xacro_object
from .gazebo import setup_custom_gazebo_models
from .ros import ROSD, ROS

# -----------------------------------------------------------------------------
# Extension of Actions, Conditions and Substitutions
# -----------------------------------------------------------------------------
from .actions import (
    BranchAction,
    IncludeLaunchFile,
    IncludeXMLLaunchFile,
    ComponentContainer,
    ContainerConfigurations,
    SetupComponentContainer,
    LogAction,
)
from .conditions import (
    ANDConditions,
    ORCondition,
    XORCondition,
    NotCondition,
)
from .substitutions import (
    TextConcat,
    AdvPathSubstitution,
    ConcatenatedPathsSubstitution,
    TernaryValue,
)
from .section import new_section
