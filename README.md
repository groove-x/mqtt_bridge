# mqtt_bridge

[![CircleCI](https://circleci.com/gh/groove-x/mqtt_bridge.svg?style=svg)](https://circleci.com/gh/groove-x/mqtt_bridge)

mqtt_bridge provides a functionality to bridge between ROS and MQTT in bidirectional.


## Principle

`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are serialized by json (or messagepack) for MQTT, and messages from MQTT are deserialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)

This limitation can be overcome by defining custom bridge class, though.


## Demo

### Prerequisites

```
$ sudo apt install python3-pip
$ sudo apt install ros-foxy-rosbridge-library
$ sudo apt install mosquitto mosquitto-clients
```

### Install python modules

```bash
$ pip3 install -r requirements.txt
```

### launch node

``` bash
$ ros2 launch mqtt_bridge demo.launch.py
```

Publish to `/ping`,

```
$ ros2 topic pub /ping std_msgs/Bool "data: true"
```

and see response to `/pong`.

```
$ ros2 topic echo /pong
data: True
---
```

Publish "hello" to `/echo`,

```
$ ros2 topic pub /echo std_msgs/String "data: 'hello'"
```

and see response to `/back`.

```
$ ros2 topic echo /back
data: hello
---
```

You can also see MQTT messages using `mosquitto_sub`

```
$ mosquitto_sub -t '#'
```

## Usage

parameter file (config.yaml):

``` yaml
mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost
    port: 1883
    keepalive: 60
n_bridge: 2 # specifying number of bridges required
bridge:
            # factory, msg_type, topic_from, topic_to
  bridge1: ["mqtt_bridge.bridge:RosToMqttBridge","std_msgs.msg:Bool","/ping","ping"]
  bridge2: ["mqtt_bridge.bridge:MqttToRosBridge","std_msgs.msg:Bool","ping","/pong"]
  # ros2 parser cannot parse the earlier config type and therefore it has to be changed as above
```

you can use any msg types like `sensor_msgs.msg:Imu`.

launch file:

``` python
# to configure node configs
config = os.path.join(
        get_package_share_directory('mqtt_bridge'),
        'config',
        'demo_params.yaml'
        )
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

### mqtt private path

If `mqtt/private_path` parameter is set, leading `~/` in MQTT topic path will be replaced by this value. For example, if `mqtt/pivate_path` is set as "device/001", MQTT path "~/value" will be converted to "device/001/value".

### serializer and deserializer

`mqtt_bridge` uses `msgpack` as a serializer by default. But you can also configure other serializers. For example, if you want to use json for serialization, add following configuration.

``` yaml
serializer: json:dumps
deserializer: json:loads
```

### bridges

You can list ROS <--> MQTT tranfer specifications in following format.

``` yaml
n_bridge: 2 # specifying number of bridges required
bridge:
            # factory, msg_type, topic_from, topic_to
  bridge1: ["mqtt_bridge.bridge:RosToMqttBridge","std_msgs.msg:Bool","/ping","ping"]
  bridge2: ["mqtt_bridge.bridge:MqttToRosBridge","std_msgs.msg:Bool","ping","/pong"]
  # ros2 parser cannot parse the earlier config type and therefore it has to be changed as above
```

* `factory`: bridge class for transfering message from ROS to MQTT, and vise versa.
* `msg_type`: ROS Message type transfering through the bridge.
* `topic_from`: topic incoming from (ROS or MQTT)
* `topic_to`: topic outgoing to (ROS or MQTT)

Also, you can create custom bridge class by inheriting `mqtt_brige.bridge.Bridge`.


## License

This software is released under the MIT License, see LICENSE.txt.
