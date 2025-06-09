# =============================================================================
#                      Arcana Python launch utils methods
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_python package
# This python file gather several methods useful for the launch module
# =============================================================================

from typing import Iterable, List
from launch import Substitution
from launch.substitutions import TextSubstitution

SubstitionsInput = Substitution | str
SubstitionsListInput = Iterable[SubstitionsInput] | SubstitionsInput | None


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


def normalize_list(x: SubstitionsListInput) -> List[Substitution]:
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
