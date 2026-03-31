import logging
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import paho.mqtt.client as mqtt


class MQTTBridge(Node):
    """
    Bridges MQTT topics to/from ROS 2 topics.

    MQTT → ROS 2:
        {prefix}/listening/set  ("on"/"off")  →  /stt_onboard/set_listening (Bool)

    ROS 2 → MQTT:
        /stt_onboard/listening_state (Bool)  →  {prefix}/listening/state ("on"/"off")
    """

    def __init__(self):
        super().__init__("mqtt_bridge")

        # ROS 2 parameters
        self.declare_parameter("broker_host", os.getenv("MQTT_BROKER_HOST", ""))
        self.declare_parameter("broker_port", int(os.getenv("MQTT_BROKER_PORT", "1883")))
        self.declare_parameter("mqtt_prefix", os.getenv("MQTT_PREFIX", "echo"))

        self.broker_host = self.get_parameter("broker_host").value
        self.broker_port = self.get_parameter("broker_port").value
        self.mqtt_prefix = self.get_parameter("mqtt_prefix").value

        self.mqtt_client = None  # Set up below only when configured

        if not self.broker_host:
            self.get_logger().info(
                "No MQTT broker host configured (MQTT_BROKER_HOST not set). "
                "MQTT bridge is disabled — running without remote control."
            )
            return

        # ROS 2 publishers / subscribers
        self.listening_pub = self.create_publisher(Bool, "/stt_onboard/set_listening", 10)
        self.state_sub = self.create_subscription(
            Bool, "/stt_onboard/listening_state", self._on_listening_state, 10
        )

        # MQTT client
        self.mqtt_client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.enable_logger(logging.getLogger(__name__))

        self.get_logger().info(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port}")
        self.mqtt_client.connect_async(self.broker_host, self.broker_port)
        self.mqtt_client.loop_start()  # Non-blocking network loop in a background thread

    # ---- MQTT callbacks ----

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker")
            topic = f"{self.mqtt_prefix}/listening/set"
            client.subscribe(topic)
            self.get_logger().info(f"Subscribed to MQTT topic: {topic}")
        else:
            self.get_logger().error(f"MQTT connection failed: {reason_code}")

    def _on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self.get_logger().warning(f"Disconnected from MQTT broker (rc={reason_code}), will auto-reconnect")

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace").strip().lower()
        self.get_logger().info(f"MQTT message: {topic} → {payload}")

        set_topic = f"{self.mqtt_prefix}/listening/set"
        if topic == set_topic:
            if payload in ("on", "1", "true"):
                self.get_logger().info("Publishing set_listening=True to ROS 2")
                self.listening_pub.publish(Bool(data=True))
            elif payload in ("off", "0", "false"):
                self.get_logger().info("Publishing set_listening=False to ROS 2")
                self.listening_pub.publish(Bool(data=False))
            else:
                self.get_logger().warning(f"Ignoring unknown payload '{payload}' on {topic}")

    # ---- ROS 2 callbacks ----

    def _on_listening_state(self, msg: Bool):
        """Forward the current listening state back to MQTT."""
        state_topic = f"{self.mqtt_prefix}/listening/state"
        payload = "on" if msg.data else "off"
        self.mqtt_client.publish(state_topic, payload, retain=True)
        self.get_logger().info(f"Published MQTT: {state_topic} → {payload}")

    # ---- Lifecycle ----

    def destroy_node(self):
        self.get_logger().info("Shutting down MQTT bridge")
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down MQTTBridge node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
