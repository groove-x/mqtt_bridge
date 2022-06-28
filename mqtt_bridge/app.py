import inject
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node

from .bridge import create_bridge
from .mqtt_client import create_private_path_extractor
from .util import lookup_object


def create_config(mqtt_client, serializer, deserializer, mqtt_private_path):
    if isinstance(serializer.value, str):
        serializer = lookup_object(serializer.value)
    if isinstance(deserializer.value, str):
        deserializer = lookup_object(deserializer.value)
    private_path_extractor = create_private_path_extractor(mqtt_private_path)
    def config(binder):
        binder.bind('serializer', serializer)
        binder.bind('deserializer', deserializer)
        binder.bind(mqtt.Client, mqtt_client)
        binder.bind('mqtt_private_path_extractor', private_path_extractor)
    return config


def mqtt_bridge_node():
    global mqtt_node 
    mqtt_node = Node('mqtt_bridge_node',
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)

    # load bridge parameters
    bridge_dict_keys = ["factory","msg_type","topic_from","topic_to"]
    bridge_params = []
    total_bridges = mqtt_node.get_parameter("n_bridges").value
    for i in range(total_bridges):
        bridge_n = str((i % total_bridges) + 1)
        bridge_param = mqtt_node.get_parameter('bridge.bridge'+bridge_n).value
        bridge_params.append(dict(zip(bridge_dict_keys,bridge_param)))

    mqtt_params ={
                    "client" :  mqtt_node.get_parameters_by_prefix("mqtt.client"),
                    "tls" : mqtt_node.get_parameters_by_prefix("mqtt.tls"),
                    "account" : mqtt_node.get_parameters_by_prefix("mqtt.account"),
                    "userdata" : mqtt_node.get_parameters_by_prefix("mqtt.userdata"),
                    "message" : mqtt_node.get_parameters_by_prefix("mqtt.message"),
                    "will"  : mqtt_node.get_parameters_by_prefix("mqtt.will")
                }
    conn_params = mqtt_node.get_parameters_by_prefix("mqtt.connection")
    for key in conn_params.keys():
        conn_params.update({key : conn_params[key].value})

    mqtt_private_path = mqtt_node.get_parameter("mqtt.private_path").value

    # create mqtt client
    mqtt_client_factory_name = mqtt_node.get_parameter_or(
        "~mqtt_client_factory", ".mqtt_client:default_mqtt_client_factory")
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(mqtt_params)

    # load serializer and deserializer
    serializer = mqtt_node.get_parameter_or('serializer', 'msgpack:dumps')
    deserializer = mqtt_node.get_parameter_or('deserializer', 'msgpack:loads')

    # dependency injection
    config = create_config(
        mqtt_client, serializer, deserializer, mqtt_private_path)
    inject.configure(config)

    # configure and connect to MQTT broker
    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect
    mqtt_client.connect(**conn_params)

    # configure bridges
    bridges = []
    for bridge_args in bridge_params:
        bridges.append(create_bridge(**bridge_args,ros_node=mqtt_node))

    # start MQTT loop
    mqtt_client.loop_start()

    try:
        rclpy.spin(mqtt_node)
    except KeyboardInterrupt:
        mqtt_node.get_logger().info('Ctrl-C detected')
        mqtt_client.disconnect()
        mqtt_client.loop_stop()

    mqtt_node.destroy_node()


def _on_connect(client, userdata, flags, response_code):
    mqtt_node.get_logger().info('MQTT connected')


def _on_disconnect(client, userdata, response_code):
    mqtt_node.get_logger().info('MQTT disconnected')


__all__ = ['mqtt_bridge_node']