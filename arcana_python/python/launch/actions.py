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
    Substitution,
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
from typing import List, Optional, Callable, Any, Dict, Iterable

from .substitutions import AdvPathSubstitution, NamespaceSubstitution, TextConcat
from .__utils import (
    normalize_null,
    normalize_list,
    ConditionInput,
    normalize_condition,
    normalize_condition_null,
    SubstitionsInput,
    SubstitionsListInput,
    LaunchFilesArguments,
)
from rclpy.node import get_logger
from rclpy.logging import LoggingSeverity


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


class LogAction(Action):
    def __init__(
        self,
        logger_name: str,
        fmt: str,
        log_lvl: LoggingSeverity = LoggingSeverity.INFO,
        args: Dict[str, SubstitionsInput | Any] = {},
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._logger = get_logger(logger_name)
        self._fmt = fmt
        self._lvl = log_lvl
        self._args = args

    def execute(self, context: LaunchContext) -> List[LaunchDescriptionEntity] | None:
        # Get all substitutions
        args: Dict[str, Any] = {}
        for k, v in self._args.items():
            if isinstance(v, Substitution):
                args[k] = v.perform(context)
            else:
                args[k] = v

        def send_log(func):
            func(self._fmt.format(**args))

        # Log it
        match self._lvl:
            case LoggingSeverity.DEBUG:
                send_log(self._logger.debug)
            case LoggingSeverity.INFO:
                send_log(self._logger.info)
            case LoggingSeverity.WARN:
                send_log(self._logger.warn)
            case LoggingSeverity.ERROR:
                send_log(self._logger.error)
            case LoggingSeverity.FATAL:
                send_log(self._logger.fatal)
        return None


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
class ComponentContainer(ComposableNodeContainer):
    """
    Short-hand to simplify the use of ComposableNodeContainer.
    """

    def __init__(
        self,
        name: SubstitionsListInput,
        namespace: SubstitionsListInput = "",
        condition: Optional[ConditionInput] = None,
        package: SubstitionsListInput = "rclcpp_components",
        executable: SubstitionsListInput = "component_container",
        **kwargs,
    ):
        super().__init__(
            name=name,
            namespace=namespace,
            condition=condition,
            package=package,
            executable=executable,
            **kwargs,
        )
        self._sub_name = name
        self._sub_ns = namespace

    def get_container(self) -> Optional[Substitution]:
        return NamespaceSubstitution(
            ns=normalize_list(self._sub_ns),
            name=TextConcat(self._sub_name),
        )


class ContainerConfigurations(GroupAction):
    """
    Declare launch arguments for setting a component container from a parent
    launch file.
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
        namespace: Optional[SubstitionsListInput] = None,
        prefix: str = "",
        condition: Optional[ConditionInput] = None,
        other_actions: Iterable[Action] = [],
        **kwargs,
    ):
        self._confs = SetupComponentContainer._ConfsName(prefix)
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
                *other_actions,
            ],
            condition=normalize_condition_null(condition),
            **kwargs,
        )


class SetupComponentContainer(ContainerConfigurations):
    """
    This action provide a way to easily setup component container.

    It also defines two launch arguments for this launch file:
        - "make_container": should the launch file create the container (useful
            when this launch file is included and the parent launch file already
            have a container)
        - "container": the name of the container
        - "container_ns": the namespace of the container

    In the case of needing to declare several containers in the same launch file,
    these configuration can be prefixed by "prefix_" via the prefix argument.
    """

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

        super().__init__(
            name=name,
            namespace=namespace,
            condition=condition,
            prefix=prefix,
            other_actions=[
                ComponentContainer(
                    name=LaunchConfiguration(self._confs.name),
                    namespace=LaunchConfiguration(self._confs.namespace),
                    package=package,
                    executable=executable,
                    condition=IfCondition(
                        LaunchConfiguration(self._confs.make_container)
                    ),
                    **container_kwargs,
                )
            ],
            **kwargs,
        )

    def get_container(self) -> NamespaceSubstitution:
        """
        Return the initialized node container.
        """
        return NamespaceSubstitution(
            LaunchConfiguration(self._confs.namespace),
            LaunchConfiguration(self._confs.name),
        )
