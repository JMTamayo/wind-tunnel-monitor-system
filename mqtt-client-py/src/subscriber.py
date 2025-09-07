import paho.mqtt.client as mqtt
import json

from config import CONFIG

TOPICS = [
    f"{CONFIG.MQTT_TOPIC_BASE}/#",
]


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"💻 Connected to broker {CONFIG.MQTT_BROKER}:{CONFIG.MQTT_PORT}")
    print(f"💻 Result code: {reason_code}")

    for topic in TOPICS:
        client.subscribe(topic)
        print(f"📥 Subscribed to: {topic}")


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"{msg.topic}: {json.dumps(payload, indent=2, ensure_ascii=False)}")
    except json.JSONDecodeError:
        print(f"{msg.topic}: {msg.payload.decode()}")


def on_disconnect(client, userdata, reason_code, properties, reason_string):
    print(f"❌ Disconnected from broker. Code: {reason_code}")


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    try:
        # Set username and password for authentication
        client.username_pw_set(CONFIG.MQTT_USERNAME, CONFIG.MQTT_PASSWORD)

        print(f"🔌 Connecting to {CONFIG.MQTT_BROKER}:{CONFIG.MQTT_PORT}...")
        client.connect(CONFIG.MQTT_BROKER, CONFIG.MQTT_PORT, CONFIG.MQTT_KEEPALIVE)

        print("🚀 Starting event loop...")
        client.loop_forever()

    except Exception as e:
        print(f"❌ Error connecting to broker: {e}")
        client.disconnect()


if __name__ == "__main__":
    main()
