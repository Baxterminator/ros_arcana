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
from launch import LaunchContext
from launch.condition import Condition
from launch.substitution import Substitution
from .__utils import SubstitionsListInput, SubstitionsInput, normalize
from launch.conditions import IfCondition, evaluate_condition_expression
from typing import List, Iterable, Text

# =============================================================================
# Argument processing
# =============================================================================

ConditionInput = Condition | SubstitionsInput
ConditionListInput = Condition | Iterable[Condition] | SubstitionsListInput


def normalize_condition(l: ConditionInput) -> Condition:
    """
    Normalize the input as a conditions to be run on

    Args:
        l (ConditionInput): a mixed type input to process

    Returns:
        Condition: a proper condition to evaluate
    """

    # Take care of the None input
    if isinstance(l, Condition):
        return l
    else:
        return IfCondition(normalize(l))


def normalize_condition_list(l: ConditionListInput) -> List[Condition]:
    """
    Normalize the input list as a list of conditions to be run on

    Args:
        l (ConditionListInput): a list of mixed inputs to process

    Returns:
        List[Condition]: a list of conditions to evaluate
    """
    # Take care of the None input
    if l is None:
        return []
    elif isinstance(l, Iterable):
        return [normalize_condition(x) for x in l]
    return [normalize_condition(l)]


# =============================================================================
# Extended conditions
# =============================================================================


class ANDConditions(Condition):
    """
    Take several conditions as input and return the AND operation of them.
    """

    def __init__(self, conditions: ConditionListInput):
        self._conditions = normalize_condition_list(conditions)
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        for c in self._conditions:
            if not c.evaluate(context):
                return False
        return True

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()


class ORCondition(Condition):
    """
    Take several conditions as input and return the OR operation of them.
    """

    def __init__(self, conditions: ConditionListInput):
        self._conditions = normalize_condition_list(conditions)
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        for c in self._conditions:
            if c.evaluate(context):
                return True
        return False

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()


class XORCondition(Condition):
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
        self._conditions = normalize_condition_list(conditions)
        self._only_one = only_one
        super().__init__(predicate=self._predicate_func)

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

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()


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


# =============================================================================
# Ternary value substitution
# =============================================================================
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
