from launch import Substitution
from launch.substitutions import TextSubstitution

SubstitionsInput = Substitution | str


def normalize(x: SubstitionsInput) -> Substitution:
    """
    Normalize input arguments as Substitutions

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
