# -*- coding: utf-8 -*-
from __future__ import absolute_import

from abc import ABCMeta, abstractmethod

import inject
import dict_to_protobuf
import paho.mqtt.client as mqtt
import rospy

from .util import lookup_object, extract_values, populate_instance, match_wildcards


def create_bridge(factory, msg_type, **kwargs):
    u""" bridge generator function

    :param (str|class) factory: Bridge class
    :param (str|class) msg_type: ROS message type
    :param str topic_from: incoming topic path
    :param str topic_to: outgoing topic path
    :param (float|None) frequency: publish frequency
    :return Bridge: bridge object
    """
    if isinstance(factory, basestring):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    if isinstance(msg_type, basestring):
        msg_type = lookup_object(msg_type)
    if not issubclass(msg_type, rospy.Message):
        raise TypeError(
            "msg_type should be rospy.Message instance or its string"
            "reprensentation")
    return factory(msg_type=msg_type, **kwargs)


class Bridge(object):
    u""" Bridge base class

    :param mqtt.Client _mqtt_client: MQTT client
    :param _serialize: message serialize callable
    :param _deserialize: message deserialize callable
    """
    __metaclass__ = ABCMeta

    _mqtt_client = inject.attr(mqtt.Client)
    _serialize = inject.attr('serializer')
    _deserialize = inject.attr('deserializer')
    _extract_private_path = inject.attr('mqtt_private_path_extractor')


class RosToMqttBridge(Bridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None, qos=2, retain=False, delete_retained_on_shutdown=False):
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)
        self._last_published = rospy.get_time()
        self._interval = 0 if frequency is None else 1.0 / frequency
        self._qos = qos
        self._retain = retain
        rospy.Subscriber(topic_from, msg_type, self._callback_ros)
        if delete_retained_on_shutdown:
            rospy.on_shutdown(self._delete_retained_on_shutdown)

    def _delete_retained_on_shutdown(self):
        self._mqtt_client.publish(topic=self._topic_to, payload="", qos=self._qos, retain=self._retain)

    def _callback_ros(self, msg):
        rospy.logdebug("ROS received from {}".format(self._topic_from))
        now = rospy.get_time()
        if now - self._last_published > self._interval:
            self._publish(msg)
            self._last_published = now

    def _publish(self, msg):
        payload = bytearray(self._serialize(extract_values(msg)))
        self._mqtt_client.publish(topic=self._topic_to, payload=payload, qos=self._qos, retain=self._retain)

class RosProtoToMqttBridge(RosToMqttBridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    """

    def __init__(self, topic_from, topic_to, msg_type, proto_type, frequency=None, qos=2, retain=False, delete_retained_on_shutdown=False):
        RosToMqttBridge.__init__(self, topic_from, topic_to, msg_type, frequency, qos, retain, delete_retained_on_shutdown)
        if isinstance(proto_type, basestring):
            proto_type = lookup_object(proto_type)
        self._proto_type = proto_type

    def _publish(self, msg):
        obj = self._proto_type()
        dict_to_protobuf.parse_dict(extract_values(msg), obj)
        payload = obj.SerializeToString()
        self._mqtt_client.publish(topic=self._topic_to, payload=payload, qos=self._qos, retain=self._retain)

class MqttToRosBridge(Bridge):
    u""" Bridge from MQTT to ROS topic

    :param str topic_from: incoming MQTT topic path
    :param str topic_to: outgoing ROS topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    :param int queue_size: ROS publisher's queue size
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None,
                 queue_size=10, wildcards=None, latch=False):
        self._topic_from = self._extract_private_path(topic_from)
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._wildcards = wildcards
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency

        self._mqtt_client.subscribe(self._topic_from)
        self._mqtt_client.message_callback_add(self._topic_from, self._callback_mqtt)
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size, latch=latch)

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        u""" callback from MQTT

        :param mqtt.Client client: MQTT client used in connection
        :param userdata: user defined data
        :param mqtt.MQTTMessage mqtt_msg: MQTT message
        """
        rospy.logdebug("MQTT received from {}".format(mqtt_msg.topic))
        now = rospy.get_time()

        if self._interval is None or now - self._last_published >= self._interval:
            try:
                ros_msg = self._create_ros_message(mqtt_msg)
                if self._wildcards is not None:
                    wildcard_matchs, hash_wildcards = match_wildcards(self._topic_from, mqtt_msg.topic)
                    wildcard_matchs.extend(hash_wildcards)
                    for wildcard, value in zip(self._wildcards, wildcard_matchs):
                        ros_msg_tmp = ros_msg
                        wildcard_split = wildcard.split(".")
                        for field in wildcard_split[:-1]:
                            ros_msg_tmp = getattr(ros_msg_tmp, field)
                        setattr(ros_msg_tmp, wildcard_split[-1], value)
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

    def _create_ros_message(self, mqtt_msg):
        u""" create ROS message from MQTT payload

        :param mqtt.Message mqtt_msg: MQTT Message
        :return rospy.Message: ROS Message
        """
        if self._msg_type._type == "std_msgs/String":
            x = self._msg_type()
            x.data = mqtt_msg.payload
            return x
        else:
            if len(mqtt_msg.payload) > 0:
                msg_dict = self._deserialize(mqtt_msg.payload)
            elif self._msg_type._type == "std_msgs/Empty":
                msg_dict = {}
            else:
                msg_dict = {}
            return populate_instance(msg_dict, self._msg_type())


__all__ = ['register_bridge_factory', 'create_bridge', 'Bridge',
           'RosToMqttBridge', 'RosProtoToMqttBridge', 'MqttToRosBridge']
