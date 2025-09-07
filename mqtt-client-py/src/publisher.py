import paho.mqtt.client as mqtt
import json
import time
import random
import threading

from config import CONFIG

# Global variables for connection management
client = None
connected = False
running = True


def on_connect(client, userdata, flags, reason_code, properties):
    global connected
    if reason_code == 0:
        connected = True
        print(f"💻 Connected to broker {CONFIG.MQTT_BROKER}:{CONFIG.MQTT_PORT}")
        print(f"💻 Result code: {reason_code}")
    else:
        connected = False
        print(f"❌ Failed to connect. Result code: {reason_code}")


def on_disconnect(client, userdata, reason_code, properties, reason_string):
    global connected
    connected = False
    print(f"❌ Disconnected from broker. Code: {reason_code}")
    if reason_string:
        print(f"❌ Reason: {reason_string}")


def on_publish(client, userdata, mid, reason_code, properties):
    if reason_code == 0:
        print(f"📤 Message published successfully (mid: {mid})")
    else:
        print(f"❌ Failed to publish message (mid: {mid}). Code: {reason_code}")


def connect_to_broker():
    """Connect to MQTT broker with retry logic"""
    global client, connected

    while running and not connected:
        try:
            print(
                f"🔌 Attempting to connect to {CONFIG.MQTT_BROKER}:{CONFIG.MQTT_PORT}..."
            )
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

            client.on_connect = on_connect
            client.on_disconnect = on_disconnect
            client.on_publish = on_publish

            client.username_pw_set(CONFIG.MQTT_USERNAME, CONFIG.MQTT_PASSWORD)
            client.connect(CONFIG.MQTT_BROKER, CONFIG.MQTT_PORT, CONFIG.MQTT_KEEPALIVE)

            client.loop_start()

            # Wait for connection to be established
            timeout = 10  # 10 seconds timeout
            start_time = time.time()
            while not connected and (time.time() - start_time) < timeout and running:
                time.sleep(0.1)

            if not connected:
                print("❌ Connection timeout. Retrying in 5 seconds...")
                client.loop_stop()
                client.disconnect()
                time.sleep(5)
            else:
                print("✅ Successfully connected to MQTT broker!")

        except Exception as e:
            print(f"❌ Error connecting to broker: {e}")
            print("🔄 Retrying in 5 seconds...")
            time.sleep(5)


def publish_command(topic, message):
    """Publish a message to the specified topic"""
    global client, connected

    if not connected or not client:
        print("❌ Not connected to broker. Cannot publish message.")
        return False

    try:
        if isinstance(message, dict):
            payload = json.dumps(message, ensure_ascii=False)
        else:
            payload = str(message)

        result = client.publish(topic, payload)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"📤 Sending to {topic}: {payload}")
            return True
        else:
            print(f"❌ Error publishing message: {result.rc}")
            return False
    except Exception as e:
        print(f"❌ Error publishing: {e}")
        return False


def message_sender():
    """Continuously send messages every 1 second"""
    global connected, running

    topics_and_messages = [
        (
            f"{CONFIG.MQTT_TOPIC_BASE}/airTemperature",
            lambda: round(random.uniform(26.0, 28.0), 2),
        ),
        (
            f"{CONFIG.MQTT_TOPIC_BASE}/airPreassure",
            lambda: round(random.uniform(0.0845, 0.0861), 4),
        ),
        (
            f"{CONFIG.MQTT_TOPIC_BASE}/airVelocity",
            lambda: round(random.uniform(13.0, 15.0), 2),
        ),
        (
            f"{CONFIG.MQTT_TOPIC_BASE}/fanFrequency",
            lambda: round(random.uniform(50, 52), 2),
        ),
    ]

    while running:
        if connected:
            for topic, message_func in topics_and_messages:
                if not running:
                    break
                publish_command(topic, message_func())
                time.sleep(0.1)  # Small delay between messages
        else:
            print("⏳ Waiting for connection...")
            time.sleep(1)

        time.sleep(1)  # Wait 1 second before next batch of messages


def connection_monitor():
    """Monitor connection and reconnect if needed"""
    global connected, running

    while running:
        if not connected:
            print("🔄 Connection lost. Attempting to reconnect...")
            connect_to_broker()
        time.sleep(2)  # Check connection every 2 seconds


def main():
    global running

    print("🚀 Starting MQTT Publisher with auto-reconnection...")
    print("Press Ctrl+C to stop")

    try:
        # Connect to broker
        connect_to_broker()

        if connected:
            # Start message sender in a separate thread
            sender_thread = threading.Thread(target=message_sender, daemon=True)
            sender_thread.start()

            # Start connection monitor in a separate thread
            monitor_thread = threading.Thread(target=connection_monitor, daemon=True)
            monitor_thread.start()

            # Keep main thread alive
            while running:
                time.sleep(1)
        else:
            print("❌ Could not establish initial connection. Exiting...")

    except KeyboardInterrupt:
        print("\n🛑 Stopping MQTT Publisher...")
        running = False

        if client:
            client.loop_stop()
            client.disconnect()

        print("✅ MQTT Publisher stopped.")


if __name__ == "__main__":
    main()
