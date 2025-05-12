from typing import Dict, List, TypeVar, Type
from dataclasses import dataclass, field
from rclpy.node import Node

__T = TypeVar("__T")
__ROS_DCL_ATT = "ros_declared"
__PARAMS_DEFAULT = {
    int: 0,
    float: 0.0,
    bool: False,
    str: "",
    list: [],
    dict: {},
}
__DATA_DEFAULT = {
    int: lambda _: 0,
    float: lambda _: 0.0,
    bool: lambda _: False,
    str: lambda _: "",
    list: lambda _: field(default_factory=list),
    dict: lambda _: field(default_factory=dict),
}


def parse_ros_params(
    node: Node,
    t: Type[__T],
    prefix: List[str] = [],
    declare_only=False,
) -> __T:
    """
    Automatically parse the ROS parameters and return a filled
    parameters object.

    Args:
        node (Node): the ROS node having the parameters
        t (Type[__T]): the type of the parameter class

    Returns:
        __T: an instance with all parameters declared and fetched
    """
    # Has the parameters been declared already ?
    declared = False
    if hasattr(t, __ROS_DCL_ATT):
        d = getattr(t, __ROS_DCL_ATT)
        if type(d) is bool:
            declared = d

    properties: Dict[str, Type] = t.__annotations__
    prefix_str = ".".join(prefix)
    if len(prefix_str) == 0:
        prefix_param = lambda p: p
    else:
        prefix_param = lambda p: f"{prefix_str}.{p}"

    # Declare parameters if not already
    if not declared:
        for prop_name, prop_type in properties.items():
            # Parse for primitive types
            if prop_type in __PARAMS_DEFAULT.keys():
                if hasattr(t, prop_name):  # If default value
                    node.declare_parameter(
                        prefix_param(prop_name),
                        getattr(t, prop_name),
                    )
                else:
                    setattr(t, prop_name, __DATA_DEFAULT[prop_type])
                    node.declare_parameter(
                        prefix_param(prop_name),
                        __PARAMS_DEFAULT[prop_type],
                    )
            # Parse for objects
            else:
                setattr(t, prop_name, field(default_factory=prop_type))
                parse_ros_params(node, prop_type, [*prefix, prop_name], True)
        setattr(t, __ROS_DCL_ATT, True)

    if declare_only:
        return None  # type:ignore

    # Create object and set attributes from parameters
    obj = dataclass(t)()
    for prop_name, prop_type in properties.items():
        if prop_type in __PARAMS_DEFAULT.keys():
            setattr(obj, prop_name, node.get_parameter(prefix_param(prop_name)).value)
        else:
            setattr(
                obj,
                prop_name,
                parse_ros_params(node, prop_type, [*prefix, prop_name]),
            )

    # Return filled object
    return obj
