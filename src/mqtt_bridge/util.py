# -*- coding: utf-8 -*-
from __future__ import absolute_import
from importlib import import_module

from rosbridge_library.internal.message_conversion import (
    extract_values, populate_instance,
)


def lookup_object(object_path, package='mqtt_bridge'):
    """ lookup object from a some.module:object_name specification. """
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj


__all__ = ['lookup_object', 'extract_values', 'populate_instance']
