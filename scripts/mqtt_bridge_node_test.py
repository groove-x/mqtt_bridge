#!/usr/bin/env python
import logging
import sys
import time
import unittest
from logging import getLogger

import msgpack
from mock import MagicMock

import rosgraph
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import Bool, String

logging.basicConfig(stream=sys.stderr)
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)


class TestMqttBridge(unittest.TestCase):

    def setUp(self):
        # mqtt
        self.mqtt_callback_ping = MagicMock()
        self.mqtt_callback_echo = MagicMock()
        self.mqttc = mqtt.Client("client-id")
        self.mqttc.connect("localhost", 1883)
        self.mqttc.message_callback_add("ping", self.mqtt_callback_ping)
        self.mqttc.message_callback_add("echo", self.mqtt_callback_echo)
        self.mqttc.subscribe("ping")
        self.mqttc.subscribe("echo")
        self.mqttc.loop_start()
        # ros
        rospy.init_node('mqtt_bridge_test_node')
        self.ros_callback_ping = MagicMock()
        self.ros_callback_pong = MagicMock()
        self.ros_callback_echo = MagicMock()
        self.ros_callback_back = MagicMock()
        self.subscriber_ping = rospy.Subscriber("/ping", Bool, self.ros_callback_ping)
        self.subscriber_pong = rospy.Subscriber("/pong", Bool, self.ros_callback_pong)
        self.subscriber_echo = rospy.Subscriber("/echo", String, self.ros_callback_echo)
        self.subscriber_back = rospy.Subscriber("/back", String, self.ros_callback_back)

    def tearDown(self):
        self.subscriber_ping.unregister()
        self.subscriber_pong.unregister()
        self.subscriber_echo.unregister()
        self.subscriber_back.unregister()
        self.mqttc.loop_stop()
        self.mqttc.disconnect()

    def get_publisher(self, topic_path, msg_type, **kwargs):
        # wait until the number of connections would be same as ros master
        pub = rospy.Publisher(topic_path, msg_type, **kwargs)
        num_subs = len(self._get_subscribers(topic_path))
        for i in range(20):
            num_cons = pub.get_num_connections()
            if num_cons == num_subs:
                return pub
            time.sleep(0.1)
        self.fail("failed to get publisher")

    def _get_subscribers(self, topic_path):
        ros_master = rosgraph.Master('/rostopic')
        topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
        state = ros_master.getSystemState()
        subs = []
        for sub in state[1]:
            if sub[0] == topic_path:
                subs.extend(sub[1])
        return subs

    def _wait_callback(self, callback_func):
        for i in range(10):
            if callback_func.called:
                return
            time.sleep(0.1)
        self.fail("callback doesn't be triggered")

    def test_ping_pong(self):
        publisher = self.get_publisher("/ping", Bool, queue_size=1)
        publisher.publish(Bool(True))
        self._wait_callback(self.ros_callback_ping)
        self.ros_callback_ping.assert_called_once_with(Bool(True))
        self._wait_callback(self.ros_callback_pong)
        self.ros_callback_pong.assert_called_once_with(Bool(True))
        self.mqtt_callback_ping.assert_called_once()
        msg = self.mqtt_callback_ping.call_args[0][2]
        self.assertEqual(msg.topic, "ping")
        self.assertEqual(msg.payload, msgpack.dumps({"data": True}))

    def test_echo_back(self):
        publisher = self.get_publisher("/echo", String, queue_size=1)
        publisher.publish(String("hello"))
        self._wait_callback(self.ros_callback_echo)
        self.ros_callback_echo.assert_called_once_with(String("hello"))
        self._wait_callback(self.ros_callback_back)
        self.ros_callback_back.assert_called_once_with(String("hello"))
        self.mqtt_callback_echo.assert_called_once()
        msg = self.mqtt_callback_echo.call_args[0][2]
        self.assertEqual(msg.topic, "echo")
        self.assertEqual(msg.payload, msgpack.dumps({"data": "hello"}))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('mqtt_bridge', 'mqtt_bridge_test', TestMqttBridge)
