# =============================================================================
#                           ROS2 launch substitutions
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file gather several substitutions methods
# =============================================================================

from launch import LaunchContext
from launch.substitution import Substitution
from typing import Text, List

from .__utils import (
    SubstitionsInput,
    SubstitionsListInput,
    normalize_list,
    normalize,
    ConditionInput,
    normalize_condition,
)
from .path import PathUtils


class TextConcat(Substitution):
    """
    Concatenate substitutions together.
    """

    def __init__(self, subs: SubstitionsListInput, separator: str = ""):
        super().__init__()
        self._substitutions = normalize_list(subs)
        self._sep = separator

    def perform(self, context: LaunchContext) -> str:
        return self._sep.join([x.perform(context) for x in self._substitutions])


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

    def __init__(self, substitution: SubstitionsInput) -> None:
        self._substitution = normalize(substitution)

    def describe(self) -> Text:
        return "AdvPathSubstitution({})".format(self._substitution.describe())

    def perform(self, context: LaunchContext) -> str:
        return PathUtils.get_object_path(self._substitution.perform(context))


class ConcatenatedPathsSubstitution(Substitution):
    """
    Substitution that concatenate several paths in one PATH-like string.
    All of the paths while be wrapped around AdvPathSubstitution objects
    """

    def __init__(self, subs: List[SubstitionsInput]) -> None:
        self._subs = normalize_list(subs)

    def describe(self) -> Text:
        return "ConcatenatedPathsSubstitution({})".format(
            "+".join([s.describe() for s in self._subs])
        )

    def perform(self, context: LaunchContext) -> Text:
        return ":".join([s.perform(context) for s in self._subs])


class TernaryValue(Substitution):
    """
    Special substitution that act upon a condition value.

    If the condition is true, performs the true_value argument.
    Otherwise, performs the false_value argument.
    """

    def __init__(
        self,
        cond: ConditionInput,
        true_value: SubstitionsInput,
        false_value: SubstitionsInput,
    ) -> None:
        super().__init__()

        # Normalize values for this substitution
        self._condition = normalize_condition(cond)
        self._true_val = normalize(true_value)
        self._false_val = normalize(false_value)

    def perform(self, context: LaunchContext) -> Text:
        """
        Perform the substitution

        Args:
            context (LaunchContext): the context of the launch

        Returns:
            Text: the result of the substitution
        """
        if self._condition.evaluate(context):
            return self._true_val.perform(context)
        return self._false_val.perform(context)
