# =============================================================================
#                         Launch Files | Actions
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file extends the default launch action possibilities.
# =============================================================================

from dataclasses import dataclass
from enum import Enum
from launch import (
    Action,
    Condition,
    LaunchContext,
    LaunchDescriptionSource,
)
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import ComposableNodeContainer
from typing import List, Optional, Callable, Any, Dict

from .conditions import ANDConditions
from .substitutions import AdvPathSubstitution
from .__utils import (
    normalize_null,
    ConditionInput,
    normalize_condition,
    normalize_condition_null,
    SubstitionsInput,
    SubstitionsListInput,
    LaunchFilesArguments,
)


# =============================================================================
# Ternary Action
# =============================================================================
class BranchAction(Action):
    """
    This action provide a way to execute one of two actions depending on the
    result of a test.
    """

    def __init__(
        self,
        test: ConditionInput,
        true_action: Action,
        false_action: Action,
        condition: Optional[Condition] = None,
        **kwargs,
    ):
        """
        Initialize a new TernaryAction object, i.e. an action branching.

        Args:
            test (Condition): the test determining which action to execute
            true_action (Action): action to execute if the test condition
                returns True
            false_action (Action): action to execute if the test condition
                return False
            condition (Optional[Condition]): the condition on which this
                branching is triggered
        """
        super().__init__(condition=condition, **kwargs)
        self._test = normalize_condition(test)
        self._true_action = true_action
        self._false_action = false_action

    def execute(
        self, context: LaunchContext
    ) -> Optional[List[LaunchDescriptionEntity]]:
        """
        Execute the branching action.

        Args:
            context (LaunchContext): the launch context

        Returns:
            List[LaunchDescriptionEntity] | None: _description_
        """
        if self._test.evaluate(context):
            return self._true_action.visit(context)
        return self._false_action.visit(context)


# =============================================================================
# Launch files
# =============================================================================
class LaunchType(Enum):
    PYTHON = 0
    XML = 1


class IncludeLaunchFile(IncludeLaunchDescription):
    """
    Short-hand to generate IncludeLaunchDescription actions.
    """

    def __init__(
        self,
        path: SubstitionsInput,
        launch_args: LaunchFilesArguments = None,
        launch_type: LaunchType = LaunchType.PYTHON,
        **kwargs,
    ):
        super().__init__(
            self._make_description(launch_type)(AdvPathSubstitution(path)),
            launch_arguments=launch_args,
            **kwargs,
        )

    def _make_description(
        self, t: LaunchType
    ) -> Callable[[SubstitionsInput], LaunchDescriptionSource]:
        match t:
            case LaunchType.PYTHON:
                return lambda k: PythonLaunchDescriptionSource(k)
            case LaunchType.XML:
                return lambda k: FrontendLaunchDescriptionSource(k)


class IncludeXMLLaunchFile(IncludeLaunchFile):
    """
    Short hand to generate IncludeLaunchDescription for XML format.
    """

    def __init__(
        self,
        path: SubstitionsInput,
        launch_args: LaunchFilesArguments = None,
        **kwargs,
    ):
        super().__init__(
            path, launch_args=launch_args, launch_type=LaunchType.XML, **kwargs
        )


# =============================================================================
# Component and container related actions
# =============================================================================
class SetupComponentContainer(GroupAction):
    """
    This action provide a way to easily setup component container.

    It also defines two launch arguments for this launch file:
        - "make_container": should the launch file create the container (useful
            when this launch file is included and the parent launch file already
            have a container)
        - "container": the name of the container
        - "container_ns": the namespace of the container
    """

    class _ConfsName:
        def __init__(self, prefix: str):
            self._prefix = f"{prefix}_" if prefix != "" else ""
            self.make_container = f"{self._prefix}make_container"
            self.name = f"{self._prefix}container"
            self.namespace = f"{self._prefix}container_ns"

    def __init__(
        self,
        name: SubstitionsListInput,
        namespace: SubstitionsListInput = "",
        condition: Optional[ConditionInput] = None,
        prefix: str = "",
        package: SubstitionsListInput = "rclcpp_components",
        executable: SubstitionsListInput = "component_container",
        container_kwargs: Dict[str, Any] = {},
        **kwargs,
    ):
        self._confs = SetupComponentContainer._ConfsName(prefix)
        self._container = ComposableNodeContainer(
            name=LaunchConfiguration(self._confs.name),
            namespace=LaunchConfiguration(self._confs.namespace),
            package=package,
            executable=executable,
            condition=IfCondition(LaunchConfiguration(self._confs.make_container)),
            **container_kwargs,
        )
        super().__init__(
            [
                DeclareLaunchArgument(
                    self._confs.make_container,
                    default_value="True",
                    description="Should the container be made ?",
                ),
                DeclareLaunchArgument(
                    self._confs.name,
                    default_value=name,
                    description="The name of the container",
                ),
                DeclareLaunchArgument(
                    self._confs.namespace,
                    default_value=namespace,
                    description="The namespace in which setup the container",
                ),
                self._container,
            ],
            condition=normalize_condition_null(condition),
            **kwargs,
        )

    def get_container(self) -> ComposableNodeContainer:
        """
        Return the initialized node container.
        """
        return self._container
