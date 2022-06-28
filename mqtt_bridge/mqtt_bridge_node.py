from rclpy.exceptions import ROSInterruptException
from mqtt_bridge.app import mqtt_bridge_node
import rclpy

def main(args=None):
    rclpy.init(args=args)
    try:
        mqtt_bridge_node()
    except ROSInterruptException:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()