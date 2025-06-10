# =============================================================================
#                      Arcana Python launch utils methods
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file gather several methods useful for the launch module
# =============================================================================

from launch import Substitution, Condition
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution
from typing import Iterable, List, Optional, Tuple

from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules

# =============================================================================
# SUBSTITUTIONS
# =============================================================================

SubstitionsInput = Substitution | str
SubstitionsListInput = Iterable[SubstitionsInput] | SubstitionsInput


def normalize(x: SubstitionsInput) -> Substitution:
    """
    Normalize input argument as Substitution

    Args:
        inp (SubstitionsInput): the input argument to normalize

    Returns:
        Substitution: the input argument as a substitution
    """
    if isinstance(x, Substitution):
        return x
    if isinstance(x, str):
        return TextSubstitution(text=x)
    raise TypeError(
        "Failed to normalize given item of type '{}', when only "
        "'str' or 'launch.Substitution' were expected.".format(type(x))
    )


def normalize_null(x: Optional[SubstitionsInput]) -> Optional[Substitution]:
    """
    Normalizes input argument as Substitution or returns None if input is None.

    Args:
        inp (SubstitionsInput, optional): the input argument to normalize

    Returns:
        Optional[Substitution]: the input argument as a substitution, or None if the input is None
    """
    if x is None:
        return None
    return normalize(x)


def normalize_list(x: Optional[SubstitionsListInput]) -> List[Substitution]:
    """
    Normalize input list values as Substitutions

    Args:
        inp (Iterable[SubstitionsInput]): the input list to normalize

    Returns:
        List[Substitution]: all elements as Substitution
    """
    if x is None:
        return []
    if isinstance(x, Iterable):
        return [normalize(s) for s in x]
    return [normalize(x)]


# =============================================================================
# CONDITIONS
# =============================================================================

ConditionInput = Condition | SubstitionsInput | bool
ConditionListInput = ConditionInput | Iterable[ConditionInput]


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
    elif type(l) is bool:
        return IfCondition(str(l))
    else:
        return IfCondition(normalize(l))


def normalize_condition_null(l: Optional[ConditionInput]) -> Optional[Condition]:
    """
    Normalize the input as a conditions to be run on. If the input is None then
    returns None.

    Args:
        l (ConditionInput, optional): a mixed type input or None to process

    Returns:
        Optional[Condition]: a proper condition to evaluate or None if the input
            was None
    """
    if l is None:
        return None
    return normalize_condition(l)


def normalize_condition_list(l: Optional[ConditionListInput]) -> List[Condition]:
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
# Others types
# =============================================================================

LaunchFilesArguments = Optional[
    Iterable[Tuple[SubstitionsListInput, SubstitionsListInput]]
]
