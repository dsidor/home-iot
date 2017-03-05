import cayenne.client
from cayenne.client import CayenneMessage, TYPE_RELATIVE_HUMIDITY, UNIT_PERCENT
import time

from drivers.hts221 import Hts221

# Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
MQTT_CLIENT_ID = '31b6c4c0-facd-11e6-a722-0dd60d860c87'
MQTT_USERNAME = 'f22327f0-a8eb-11e6-a7c1-b395fc8a1540'
MQTT_PASSWORD = '8d58b83e86d1ef3a0c676af8eff332963c8f190c'


# The callback for when a message is received from Cayenne.
def on_message(message):
    """
    :param CayenneMessage message:
    :return:
    """
    print("message received: " + str(message))
    print 'I should set channel', message.channel, 'to', message.value
    # If there is an error processing the message return an error string, otherwise return nothing.
    return None


client = cayenne.client.CayenneMQTTClient()
client.on_message = on_message
client.begin(MQTT_USERNAME, MQTT_PASSWORD, MQTT_CLIENT_ID)

i = 0
timestamp = 0
channels = {
    'temperature': 1,
    'humidity': 2,
}
PERIOD_S = 60

while True:
    client.loop()

    time.sleep(PERIOD_S)
    sensor = Hts221()
    temperature, humidity = sensor.read_temperature_and_humidity()
    client.celsiusWrite(channels['temperature'], temperature)
    client.virtualWrite(channels['humidity'], humidity, TYPE_RELATIVE_HUMIDITY, UNIT_PERCENT)
