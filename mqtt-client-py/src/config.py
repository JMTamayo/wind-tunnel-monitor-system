from pydantic_settings import BaseSettings


class Config(BaseSettings):
    MQTT_BROKER: str
    MQTT_USERNAME: str
    MQTT_PASSWORD: str
    MQTT_PORT: int = 1883
    MQTT_KEEPALIVE: int = 60

    MQTT_TOPIC_BASE: str


CONFIG: Config = Config()
