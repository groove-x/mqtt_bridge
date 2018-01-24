# -*- coding: utf-8 -*-
from __future__ import absolute_import
from importlib import import_module

from rosbridge_library.internal import message_conversion


def lookup_object(object_path, package='mqtt_bridge'):
    """ lookup object from a some.module:object_name specification. """
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj


def monkey_patch_message_conversion():
    u""" modify _to_primitive_inst to distinct unicode and str conversion """
    from rosbridge_library.internal.message_conversion import (
        type_map, primitive_types, string_types, FieldTypeMismatchException,
    )
    def _to_primitive_inst(msg, rostype, roottype, stack):
        # Typecheck the msg
        msgtype = type(msg)
        if msgtype in primitive_types and rostype in type_map[msgtype.__name__]:
            return msg
        elif msgtype is unicode and rostype in type_map[msgtype.__name__]:
            return msg.encode("utf-8", "ignore")
        elif msgtype is str and rostype in type_map[msgtype.__name__]:
            return msg.decode("utf-8").encode("utf-8", "ignore")
        raise FieldTypeMismatchException(roottype, stack, rostype, msgtype)
    message_conversion._to_primitive_inst = _to_primitive_inst


monkey_patch_message_conversion()


def extract_values(inst):
    import rospy
    if isinstance(inst, rospy.Time):
        rostype = "time"
    elif hasattr(inst, "_type"):
        rostype = getattr(inst, "_type", None)
    else:
        raise InvalidMessageException()
    return message_conversion._from_inst(inst, rostype)


def populate_instance(msg, inst):
    import rospy
    if isinstance(inst, rospy.Time):
        rostype = "time"
    elif hasattr(inst, "_type"):
        rostype = getattr(inst, "_type", None)
    else:
        raise message_conversion.InvalidMessageException(inst)
    return message_conversion._to_inst(msg, rostype, rostype, inst)


__all__ = ['lookup_object', 'extract_values', 'populate_instance']
