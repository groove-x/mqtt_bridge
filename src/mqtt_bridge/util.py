from importlib import import_module
from typing import Any, Callable, Dict

import rospy
from rosbridge_library.internal import message_conversion

def lookup_object(object_path: str, package: str='mqtt_bridge') -> Any:
    """ lookup object from a some.module:object_name specification. """
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj


extract_values = message_conversion.extract_values  # type: Callable[[rospy.Message], Dict]
populate_instance = message_conversion.populate_instance  # type: Callable[[Dict, rospy.Message], rospy.Message]


__all__ = ['lookup_object', 'extract_values', 'populate_instance']
