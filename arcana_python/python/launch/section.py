# =============================================================================
#                     Launch Files | General Purpose section
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file provide a section class that allow to group modifications.
# =============================================================================

from enum import Enum
from launch import LaunchDescription, Action, Condition
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from typing import Optional, Iterable, Any, Callable, Dict

from .actions import IncludeLaunchFile, LaunchType, SetupComponentContainer
from .conditions import ANDConditions, ExtendableCondition
from .substitutions import AdvPathSubstitution
from .__utils import (
    SubstitionsInput,
    SubstitionsListInput,
    ConditionInput,
    SomeParameters,
    SomeRemapRules,
    LaunchFilesArguments,
    normalize,
    normalize_condition,
)


class __ArgsNames(Enum):
    PARAMETERS = "parameters"
    NAMESPACE = "namespace"
    CONDITION = "condition"


class new_section:
    """
    General purpose section to assign common parameters to several actions.

    The modifications the section can spread are:
        - config: configuration file to use for all nodes and composable nodes
        - namespace: a common namespace for all nodes and composable nodes
        - condition: a condition to apply to all actions
    """

    # =========================================================================
    # Section init
    # =========================================================================

    def __init__(
        self,
        ld: "LaunchDescription | new_section",
        config: Optional[SubstitionsInput] = None,
        namespace: Optional[SubstitionsInput] = None,
        condition: Optional[ConditionInput] = None,
    ):
        self._ld = ld
        self._config_file = new_section.__set_config_file(ld, config)
        self._namespace = new_section.__set_namespace(ld, namespace)
        self._condition = new_section.__set_condition(ld, condition)

    def __enter__(self) -> "new_section":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        return

    # =========================================================================
    # High level interface
    # =========================================================================

    def add_action(self, action: Action) -> None:
        """
        Add an action to the launch description.

        Args:
            action (Action): the action to add
        """
        self._ld.add_action(action)

    def add_node(
        self,
        executable: SubstitionsListInput,
        package: Optional[SubstitionsListInput] = None,
        name: Optional[SubstitionsListInput] = None,
        namespace: Optional[SubstitionsListInput] = None,
        exec_name: Optional[SubstitionsListInput] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        arguments: Optional[Iterable[SubstitionsInput]] = None,
        ros_arguments: Optional[Iterable[SubstitionsInput]] = None,
        condition: Optional[ConditionInput] = None,
        **kwargs,
    ) -> None:
        """
        Add a node to the launch description.
        Forwards the arguments to the Node action.
        """
        self._ld.add_action(Node(**self._modify_params(locals())))

    def include_launch(
        self,
        path: SubstitionsInput,
        launch_args: LaunchFilesArguments = None,
        launch_type: LaunchType = LaunchType.PYTHON,
        condition: Optional[ConditionInput] = None,
        **kwargs,
    ) -> None:
        """
        Include a launch file in the launch description

        Args:
            launch_file (SubstitionsInput): the path to the launch file
        """
        self._ld.add_action(
            IncludeLaunchFile(
                **self._modify_params(locals()),
            )
        )

    def add_component(
        self,
        package: SubstitionsListInput,
        plugin: SubstitionsListInput,
        container: SubstitionsListInput | ComposableNodeContainer,
        name: Optional[SubstitionsListInput] = None,
        namespace: Optional[SubstitionsListInput] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        extra_arguments: Optional[SomeParameters] = None,
        condition: Optional[ConditionInput] = None,
    ) -> None:
        """
        Add a configured component to the launch description.
        """
        self._ld.add_action(
            LoadComposableNodes(
                composable_node_descriptions=[
                    ComposableNode(**self._modify_params(locals()))
                ],
                target_container=container,
            )
        )

    def setup_container(
        self,
        name: SubstitionsListInput,
        namespace: Optional[SubstitionsListInput] = None,
        condition: Optional[ConditionInput] = None,
        prefix: Optional[str] = None,
        package: Optional[SubstitionsListInput] = None,
        executable: Optional[SubstitionsListInput] = None,
        container_kwargs: Dict[str, Any] = {},
        **kwargs,
    ) -> ComposableNodeContainer:
        """
        Setup a new container in this section and apply .
        """
        action = SetupComponentContainer(**self._modify_params(locals()))
        self._ld.add_action(action)
        return action.get_container()

    # =========================================================================
    # Set object parameters
    # =========================================================================
    @staticmethod
    def __set_config_file(
        ld: "LaunchDescription|new_section",
        config: Optional[SubstitionsInput],
    ) -> Optional[AdvPathSubstitution]:
        """
        Returns a configured config file substitution if config is not None,
        returns None otherwise.
        Allow parent configuration forwarding.
        """
        if config is not None:
            return AdvPathSubstitution(config)
        if isinstance(ld, LaunchDescription):
            return None
        return ld._config_file

    @staticmethod
    def __set_namespace(
        ld: "LaunchDescription|new_section",
        ns: Optional[SubstitionsInput],
    ) -> Optional[SubstitionsInput]:
        """
        Returns a configured namespace substitution if ns is not None,
        returns None otherwise.
        Allow parent configuration forwarding.
        """
        if ns is not None:
            return ns
        if isinstance(ld, LaunchDescription):
            return None
        return ld._namespace

    @staticmethod
    def __set_condition(
        ld: "LaunchDescription|new_section",
        condition: Optional[ConditionInput],
    ) -> Optional[Condition]:
        """
        Returns a configured condition if input is not None,
        return None otherwise.
        Allow parent configuration forwarding.

        If both parent and input are valid conditions, than merge
        conditions.
        """
        has_parent = not isinstance(ld, LaunchDescription)
        if has_parent:
            return new_section.__merge_conditions(ld._condition, condition)
        elif condition is not None:
            return normalize_condition(condition)
        return None

    # =========================================================================
    # Modify the actions arguments
    # =========================================================================

    def _modify_params(self, args: dict) -> dict:
        """
        Modify the given params according to the modification rules:

            - Addidition of a parameter file on key "parameters"
            - Modification of a namespace on key "namespace"
        """

        def apply_modification(
            val: Optional[Any],
            key: __ArgsNames,
            clbk: Callable[[dict, str], None],
        ) -> None:
            """
            Callback macro to apply the several modifications.
            """
            if val is not None and key.value in new_args.keys():
                clbk(new_args, key.value)

        new_args = args.copy()

        # Apply all modification rules
        apply_modification(
            self._config_file,
            __ArgsNames.PARAMETERS,
            self.__modify_params,
        )
        apply_modification(
            self._namespace,
            __ArgsNames.NAMESPACE,
            self.__modify_namespace,
        )
        self.__apply_condition(new_args, __ArgsNames.CONDITION.value)
        return self.__remove_nones(new_args)

    def __modify_params(self, args: dict, key: str):
        """
        Modify the given arguments dictionnary to add the config file in the
        parameters.
        """
        if args[key] is not None:
            args[key] = [self._config_file, *(args[key])]
        else:
            args[key] = [self._config_file]

    def __modify_namespace(self, args: dict, key: str):
        """
        Modify the given arguments dictionnary to modify the namespace if the
        argument is None only (allow overriding).
        """
        if args[key] is None:
            args[key] = self._namespace

    def __apply_condition(self, args: dict, key: str):
        """
        Modify and normalize condition input
        """
        key_exist = key in args.keys()

        args[key] = self.__merge_conditions(
            self._condition,
            args[key] if key_exist else None,
        )

    @staticmethod
    def __merge_conditions(
        condA: Optional[ConditionInput],
        condB: Optional[ConditionInput],
    ) -> Optional[Condition]:
        """
        Merge two condition together.
        """
        if condA is None:
            if condB is None:  # No condition given for A or B
                return None
            return normalize_condition(condB)

        # If condA exist but not condB
        if condB is None:
            return normalize_condition(condA)

        # Else merge them
        if isinstance(condA, ExtendableCondition):
            return condA.extend(condB)
        elif isinstance(condB, ExtendableCondition):
            return condB.extend(condA)
        else:
            return ANDConditions([condA, condB])

    @staticmethod
    def __remove_nones(args: dict) -> dict:
        """
        Remove all None arguments from the dictionnary.
        """
        return {k: v for k, v in args.items() if v is not None}
