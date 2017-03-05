import smbus


class Hts221(object):
    _hts221_address = 0x5F
    _average_configuration_register = 0x10
    _control_register1 = 0x20

    def __init__(self):
        self._bus = smbus.SMBus(1)
        self._configure_samples_number()
        self._enable_and_set_continues_update_data_rate_1hz()
        self._read_humidity_calibration()
        self._read_temperature_calibration()

    def _configure_samples_number(self):
        # 0x1B(27) - Temperature average samples = 256, Humidity average samples = 512
        self._write_data(self._average_configuration_register, 0x1B)

    def _enable_and_set_continues_update_data_rate_1hz(self):
        self._write_data(self._control_register1, 0x85)

    def _write_data(self, address, data):
        self._bus.write_byte_data(self._hts221_address, address, data)

    def _read_data(self, address):
        return self._bus.read_byte_data(self._hts221_address, address)

    def _read_humidity_calibration(self):
        self.H0 = self._read_data(0x30) / 2
        self.H1 = self._read_data(0x31) / 2
        self.H2 = (self._read_data(0x37) * 256) + self._read_data(0x36)
        self.H3 = (self._read_data(0x3B) * 256) + self._read_data(0x3A)

    def _read_temperature_calibration(self):
        msbits = self._read_data(0x35)
        self.T0 = (msbits & 0x03) * 256 + self._read_data(0x32)
        self.T1 = (msbits & 0x0C) * 64 + self._read_data(0x33)
        self.T2 = self._read_data(0x3D) * 256 + self._read_data(0x3C)
        self.T3 = self._read_data(0x3F) * 256 + self._read_data(0x3E)

    def read_temperature_and_humidity(self):
        # humidity msb, humidity lsb, temp msb, temp lsb
        # Read data back from 0x28(40) with command register 0x80(128)
        data = self._bus.read_i2c_block_data(self._hts221_address, 0x28 | 0x80, 4)

        raw_humidity = (data[1] * 256) + data[0]
        humidity = ((1.0 * self.H1) - (1.0 * self.H0)) * (1.0 * raw_humidity - 1.0 * self.H2) / (
            1.0 * self.H3 - 1.0 * self.H2) + (1.0 * self.H0)
        raw_temp = (data[3] * 256) + data[2]
        if raw_temp > 32767:
            raw_temp -= 65536
        temperature = (self.T1 - self.T0) / 8.0 * (raw_temp - self.T2) / (self.T3 - self.T2) + (self.T0 / 8.0)

        return temperature, humidity

if __name__ == '__main__':
    sensor = Hts221()
    temp, humi = sensor.read_temperature_and_humidity()
    print "Temperature: %.2f C" % temp
    print "Relative Humidity : %.2f %%" % humi
