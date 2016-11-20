# mqtt_bridge

mqtt_bridge provides a functionality to bridge between ROS and MQTT in bidirectional.


## Principle

`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are selialized by json (or messagepack) for MQTT, and messages from MQTT are deselialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)

However, this limitation can be overcome by defining custom bridge class as described later.


## Usage

Prepare parameter file (config.yaml).

``` yaml
mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost
    port: 1883
    keepalive: 60
bridge:
  # ping pong
  - factory: "mqtt_bridge.bridge:RosToMqttBridge"
    msg_type: "std_msgs.msg:Bool"
    topic_from: "/ping"
    topic_to: "/ping"
  - factory: "mqtt_bridge.bridge:MqttToRosBridge"
    msg_type: "std_msgs.msg:Bool"
    topic_from: "/ping"
    topic_to: "/pong"
```

and launch mqtt_bridge_node with these parameters.

``` xml
<launch>
  <node name="mqtt_bridge" pkg="mqtt_bridge" type="mqtt_bridge_node.py" output="screen">
    <rosparam file="/path/to/config.yaml" command="load" />
  </node>
</launch>
```


## Configuration

### mqtt

Parameters under `mqtt` section are used for creating paho's `mqtt.Client` and its configuration.

#### subsections

* `client`: used for `mqtt.Client` constructor
* `tls`: used for tls configuration
* `account`: used for username and password configuration
* `message`: used for MQTT message configuration
* `userdata`: used for MQTT userdata configuration
* `will`: used for MQTT's will configuration

See `mqtt_bridge.mqtt_client` for detail.

### selializer and deselializer

`mqtt_bridge` uses `json` as a selializer in default. But you can also configure other selializers. For example, if you want to use messagepack for selialization, add following configuration.

``` yaml
selializer: "msgpack:dumps"
deselializer: "msgpack:loads"
```

### bridges

You can list ROS <--> MQTT tranfer specifications in following format.

``` yaml
bridge:
  # ping pong
  - factory: "mqtt_bridge.bridge:RosToMqttBridge"
    msg_type: "std_msgs.msg:Bool"
    topic_from: "/ping"
    topic_to: "/ping"
  - factory: "mqtt_bridge.bridge:MqttToRosBridge"
    msg_type: "std_msgs.msg:Bool"
    topic_from: "/ping"
    topic_to: "/pong"
```

* `factory`: bridge class for transfering message from ROS to MQTT, and vise versa.
* `msg_type`: ROS Message type transfering through the bridge.
* `topic_from`: topic incoming from (ROS or MQTT)
* `topic_to`: topic outgoing to (ROS or MQTT)

Also, you can create custom bridge class by inheriting `mqtt_brige.bridge.Bridge` and specify it in config.


## License

This software is released under the MIT License, see LICENSE.txt.


## Versions

### 0.1.0

- Initial release
