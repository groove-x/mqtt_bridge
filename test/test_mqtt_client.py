from mqtt_bridge.mqtt_client import create_private_path_extractor


def test_create_private_path_extractor():
    extract = create_private_path_extractor("mqtt/server1")
    assert extract("~/some/path") == "mqtt/server1/some/path"
