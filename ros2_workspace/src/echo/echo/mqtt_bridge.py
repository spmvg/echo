import importlib
import json
import os

import rclpy
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from std_msgs.msg import Bool

import paho.mqtt.client as mqtt


class MQTTBridge(Node):
    """
    Bridges MQTT topics to/from ROS 2 topics.

    MQTT → ROS 2 (built-in):
        {prefix}/listening/set  ("on"/"off")  →  /stt_onboard/set_listening (Bool)

    ROS 2 → MQTT (built-in):
        /stt_onboard/listening_state (Bool)  →  {prefix}/listening/state ("on"/"off")

    MQTT → ROS 2 (generic relay):
        {prefix}/ros2/publish  (JSON)  →  any allowed ROS 2 topic

        JSON payload format:
            {
                "topic": "/some/ros2/topic",
                "data": { <message fields> }
            }

        Configure allowed topics via the MQTT_ALLOWED_TOPICS environment variable
        (JSON array, evaluated once at startup):

            MQTT_ALLOWED_TOPICS='[
                {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
                {"topic": "/my_topic", "type": "std_msgs/msg/String"}
            ]'

        Topics not present in this list are silently rejected.
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

        # ROS 2 publishers / subscribers (built-in echo topics)
        self.listening_pub = self.create_publisher(Bool, "/stt_onboard/set_listening", 10)
        self.state_sub = self.create_subscription(
            Bool, "/stt_onboard/listening_state", self._on_listening_state, 10
        )

        # Generic relay: parse allowed topics and pre-create publishers
        self.allowed_topics: dict[str, str] = self._parse_allowed_topics()
        self._relay_publishers: dict[str, rclpy.publisher.Publisher] = {}
        for ros2_topic, msg_type in self.allowed_topics.items():
            try:
                MsgClass = self._get_message_class(msg_type)
                self._relay_publishers[ros2_topic] = self.create_publisher(MsgClass, ros2_topic, 10)
                self.get_logger().info(f"Relay publisher ready: {ros2_topic} ({msg_type})")
            except Exception as exc:
                self.get_logger().error(
                    f"Relay: failed to create publisher for {ros2_topic} ({msg_type}): {exc}"
                )

        # MQTT client
        # Note: enable_logger() is intentionally omitted — rclpy replaces the standard
        # Python logger with RcutilsLogger, whose .log() signature is incompatible with
        # the variadic-args format that paho uses internally, causing a TypeError.
        self.mqtt_client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message

        self.get_logger().info(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port}")
        self.mqtt_client.connect_async(self.broker_host, self.broker_port)
        self.mqtt_client.loop_start()  # Non-blocking network loop in a background thread

        # Publish set_listening=True once on startup
        self.startup_timer = self.create_timer(0.5, self._set_listening_on_startup)

    # ---- Startup ----

    def _set_listening_on_startup(self):
        self.get_logger().info("Set listening true on startup")
        self.listening_pub.publish(Bool(data=True))
        self.startup_timer.cancel()

    # ---- Helpers ----

    def _parse_allowed_topics(self) -> dict[str, str]:
        """Parse MQTT_ALLOWED_TOPICS into a {ros2_topic: msg_type} dict."""
        raw = os.getenv("MQTT_ALLOWED_TOPICS", "").strip()
        if not raw:
            return {}
        try:
            entries = json.loads(raw)
            result = {item["topic"]: item["type"] for item in entries}
            self.get_logger().info(
                f"Relay: {len(result)} allowed topic(s): {list(result.keys())}"
            )
            return result
        except Exception as exc:
            self.get_logger().error(f"Failed to parse MQTT_ALLOWED_TOPICS: {exc}")
            return {}

    @staticmethod
    def _get_message_class(msg_type: str):
        """
        Dynamically import and return a ROS 2 message class.

        Accepted formats:
            "std_msgs/msg/String"   (canonical, preferred)
            "std_msgs/String"       (short form)
        """
        parts = msg_type.split("/")
        if len(parts) == 3:
            pkg, _, cls_name = parts
        elif len(parts) == 2:
            pkg, cls_name = parts
        else:
            raise ValueError(f"Unsupported message type format: {msg_type!r}")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, cls_name)

    # ---- MQTT callbacks ----

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Connected to MQTT broker")

            # Built-in listening control topic
            listen_topic = f"{self.mqtt_prefix}/listening/set"
            client.subscribe(listen_topic)
            self.get_logger().info(f"Subscribed to MQTT topic: {listen_topic}")

            # Generic relay topic (only subscribe when allowed topics are configured)
            if self._relay_publishers:
                relay_topic = f"{self.mqtt_prefix}/ros2/publish"
                client.subscribe(relay_topic)
                self.get_logger().info(f"Subscribed to MQTT relay topic: {relay_topic}")
        else:
            self.get_logger().error(f"MQTT connection failed: {reason_code}")

    def _on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self.get_logger().warning(f"Disconnected from MQTT broker (rc={reason_code}), will auto-reconnect")

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic

        set_topic = f"{self.mqtt_prefix}/listening/set"
        relay_topic = f"{self.mqtt_prefix}/ros2/publish"

        if topic == set_topic:
            payload = msg.payload.decode("utf-8", errors="replace").strip().lower()
            self.get_logger().info(f"MQTT message: {topic} → {payload}")
            if payload in ("on", "1", "true"):
                self.get_logger().info("Publishing set_listening=True to ROS 2")
                self.listening_pub.publish(Bool(data=True))
            elif payload in ("off", "0", "false"):
                self.get_logger().info("Publishing set_listening=False to ROS 2")
                self.listening_pub.publish(Bool(data=False))
            else:
                self.get_logger().warning(f"Ignoring unknown payload '{payload}' on {topic}")

        elif topic == relay_topic:
            self._handle_ros2_relay(msg.payload)

    def _handle_ros2_relay(self, payload_bytes: bytes) -> None:
        """
        Forward a generic MQTT message to a ROS 2 topic.

        Expected JSON payload::

            {
                "topic": "/some/ros2/topic",
                "data": { <message fields as a dict> }
            }

        For simple single-field messages (e.g. std_msgs/msg/String) ``data``
        may also be a plain scalar, which will be assigned to the ``data``
        field directly.
        """
        try:
            envelope = json.loads(payload_bytes.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Relay: invalid JSON payload: {exc}")
            return

        ros2_topic = envelope.get("topic")
        msg_data = envelope.get("data", {})

        if not ros2_topic:
            self.get_logger().error("Relay: missing 'topic' field in JSON payload")
            return

        pub = self._relay_publishers.get(ros2_topic)
        if pub is None:
            self.get_logger().warning(
                f"Relay: topic '{ros2_topic}' is not in the allowed list — ignoring. "
                f"Allowed: {list(self._relay_publishers.keys())}"
            )
            return

        msg_type = self.allowed_topics[ros2_topic]
        try:
            MsgClass = self._get_message_class(msg_type)
            ros_msg = MsgClass()
            if isinstance(msg_data, dict):
                set_message_fields(ros_msg, msg_data)
            else:
                # Scalar shorthand for messages with a single `data` field
                ros_msg.data = msg_data
            pub.publish(ros_msg)
            self.get_logger().info(f"Relay: published to {ros2_topic} ({msg_type})")
        except Exception as exc:
            self.get_logger().error(f"Relay: failed to publish to {ros2_topic}: {exc}")

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
