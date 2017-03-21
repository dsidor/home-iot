import time
import logging

from blynk.BlynkLib import BlynkConnection, BlynkClient


auth_token = '51faa2e21d694800a10672adc94861e7'

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger('blynk')
log.setLevel(logging.DEBUG)


def read_temperature(pin, blynk_client):
    return 15


def read_humidity(pin, blynk_client):
    return 42


def read_slider(pin, blynk_client):
    return 50


def write_slider(value, pin, blynk_client):
    """
    :param int pin:
    :param value:
    :param BlynkClient blynk_client:
    :return:
    """
    blynk_client.virtual_write(1, value)


def main():
    conn = BlynkConnection(auth_token)
    blynk = BlynkClient(conn)
    blynk.add_virtual_pin(0, read_temperature)
    blynk.add_virtual_pin(1, read_humidity)
    blynk.add_virtual_pin(2, write=write_slider)
    blynk.run()


if __name__ == '__main__':
    main()
