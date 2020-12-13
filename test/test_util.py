from mqtt_bridge.bridge import RosToMqttBridge
from mqtt_bridge.util import extract_values, populate_instance, lookup_object
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header


def test_lookup_object():
    obj = lookup_object('mqtt_bridge.bridge:RosToMqttBridge')
    assert obj == RosToMqttBridge


def test_extract_values():
    msg = Temperature(
        header=Header(),
        temperature=25.2,
        variance=0.0,
    )
    expected = {'header': {'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': '', 'seq': 0}, 'temperature': 25.2, 'variance': 0.0}
    actual = extract_values(msg)
    assert expected == actual


def test_populate_instance():
    msg_dict = {'header': {'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': '', 'seq': 0}, 'temperature': 25.2, 'variance': 0.0}
    msg = Temperature()
    populate_instance(msg_dict, msg)
    assert msg.temperature == 25.2
    assert msg.variance == 0.0
