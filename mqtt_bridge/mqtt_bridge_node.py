from rclpy.exceptions import ROSInterruptException
from mqtt_bridge.app import mqtt_bridge_node

try:
    mqtt_bridge_node()
except ROSInterruptException:
    pass