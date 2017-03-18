import time
import logging

import BlynkLib


auth_token = '51faa2e21d694800a10672adc94861e7'

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger('blynk')
log.setLevel(logging.DEBUG)


def read_temperature(pin, state, blynk_ref):
    log.debug('Reading {} pin'.format(pin))
    return 15


def read_humidity(pin, state, blynk_ref):
    log.debug('Reading {} pin'.format(pin))
    return 42


def read_slider(pin, state, blynk_ref):
    log.debug('Reading {} pin'.format(pin))
    return 50


def write_slider(value, pin, state, blynk_ref):
    log.debug('Writing {} pin to {}'.format(pin, value))


def main():
    blynk = BlynkLib.Blynk(auth_token)

    blynk.add_virtual_pin(0, read_temperature)
    blynk.add_virtual_pin(1, read_humidity)
    blynk.add_virtual_pin(2, write=write_slider)

    blynk.virtual_write(2, 120)
    blynk.run()


if __name__ == '__main__':
    main()
