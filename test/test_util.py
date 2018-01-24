# -*- coding: utf-8 -*-
from __future__ import absolute_import
from nose.tools import ok_, eq_

from mqtt_bridge.util import (
    lookup_object, extract_values, populate_instance,
)


def test_lookup_object():
    obj = lookup_object('mqtt_bridge.bridge:Bridge')
    from mqtt_bridge.bridge import Bridge
    eq_(obj, Bridge)


class TestExtractValues(object):

    def test_vector3(self):
        from geometry_msgs.msg import Vector3
        msg = Vector3(1, 2, 3)
        obj = extract_values(msg)
        eq_(obj, {'x': 1, 'y': 2, 'z': 3})

    def test_time(self):
        from rospy import Time
        msg = Time(12345, 6789)
        obj = extract_values(msg)
        eq_(obj, {'secs': 12345, 'nsecs': 6789})


class TestPopulateInstance(object):

    def test_vector3(self):
        from geometry_msgs.msg import Vector3
        val = {'x': 1, 'y': 2, 'z': 3}
        msg = populate_instance(val, Vector3())
        eq_(msg, Vector3(1, 2, 3))

    def test_time(self):
        from rospy import Time
        val = {'secs': 12345, 'nsecs': 6789}
        msg = populate_instance(val, Time())
        eq_(msg, Time(12345, 6789))
