# =============================================================================
#                         Launch Files | Conditions
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file extends the default launch conditions possibilities by
# combining and inversing them.
# =============================================================================
from abc import ABC, abstractmethod
from launch import LaunchContext
from launch.condition import Condition
from typing import Text, Iterable

from .__utils import (
    ConditionListInput,
    ConditionInput,
    normalize_condition,
    normalize_condition_list,
)

# =============================================================================
# Extended conditions
# =============================================================================


class ExtendableCondition(Condition, ABC):
    """
    Interface for extandable conditions which support extension
    """

    def __init__(self, conditions: ConditionListInput):
        self._conditions = normalize_condition_list(conditions)
        super().__init__(predicate=self._predicate_func)

    @abstractmethod
    def _predicate_func(self, context: LaunchContext) -> bool:
        return False

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()

    def extend(self, condition: ConditionListInput) -> "ExtendableCondition":
        """
        Create a new ExtendableCondition with additional conditions.

        This prevent too many nested Condition objects.
        """
        if isinstance(condition, Iterable):
            return ANDConditions([*self._conditions, *condition])
        return ANDConditions([*self._conditions, condition])


class ANDConditions(ExtendableCondition):
    """
    Take several conditions as input and return the AND operation of them.
    """

    def __init__(self, conditions: ConditionListInput):
        super().__init__(conditions)

    def _predicate_func(self, context: LaunchContext) -> bool:
        for c in self._conditions:
            if not c.evaluate(context):
                return False
        return True


class ORCondition(ExtendableCondition):
    """
    Take several conditions as input and return the OR operation of them.
    """

    def __init__(self, conditions: ConditionListInput):
        super().__init__(conditions)

    def _predicate_func(self, context: LaunchContext) -> bool:
        for c in self._conditions:
            if c.evaluate(context):
                return True
        return False


class XORCondition(ExtendableCondition):
    """
    Take several conditions as input and return the XOR operation of them.

    This implementation of XOR for more than 2 inputs is:

        - If input "only_one" is set to False, it will return True when an
            odd number of condition equals to True (chaining XOR on the
            arguments)
        - If input "only_one" is set to True, it will return True when exactly
            one condition equals to True (more in the spirit of the XOR
            operator)
    """

    def __init__(self, conditions: ConditionListInput, only_one: bool = False):
        self._only_one = only_one
        super().__init__(conditions)

    def _predicate_func(self, context: LaunchContext) -> bool:
        if self._only_one:
            val = False
            for c in self._conditions:
                if c.evaluate(context):
                    if val:
                        return False
                    val = True
            return False
        else:
            val = False
            for c in self._conditions:
                if c.evaluate(context):
                    val = not val
            return val


class NotCondition(Condition):
    """
    This special condtion only inverse the value of the argument.
    """

    def __init__(self, cond: ConditionInput):
        self._cond = normalize_condition(cond)
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return not self._cond.evaluate(context)

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()
