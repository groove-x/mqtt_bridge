#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from mqtt_bridge.app import mqtt_bridge_node


try:
    mqtt_bridge_node()
except rospy.ROSInterruptException:
    pass
