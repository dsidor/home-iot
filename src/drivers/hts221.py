import smbus


def _to_signed(value):
    if value > 32767:
        return value - 65536
    else:
        return value


class Hts221(object):
    _hts221_address = 0x5F
    _AV_CONF = 0x10
    _CTRL_REG1 = 0x20

    def __init__(self):
        self._bus = smbus.SMBus(1)
        self._configure_samples_number()
        self._enable_and_set_continues_update_data_rate_1hz()
        self._read_calibration()

    def _configure_samples_number(self):
        # 0x3F(0b00111111) - Temperature average samples = 256, Humidity average samples = 512
        self._write_data(self._AV_CONF, 0x3F)

    def _enable_and_set_continues_update_data_rate_1hz(self):
        # 0b10000000 - enable | 0b00000100 - continues | 0b00000001 - 1Hz
        self._write_data(self._CTRL_REG1, 0x85)

    def _write_data(self, address, data):
        self._bus.write_byte_data(self._hts221_address, address, data)

    def _read_data(self, address):
        return self._bus.read_byte_data(self._hts221_address, address)

    # noinspection PyPep8Naming
    def _read_calibration(self):
        H0_rH_x2 = self._read_data(0x30)
        H0_rH = H0_rH_x2 / 2.0
        H1_rH_x2 = self._read_data(0x31)
        H1_rH = H1_rH_x2 / 2.0
        H0_T0_OUT_lsb = self._read_data(0x36)
        H0_T0_OUT_msb = self._read_data(0x37)
        H0_T0_OUT = float(_to_signed((H0_T0_OUT_msb << 8) + H0_T0_OUT_lsb))
        H1_T0_OUT_lsb = self._read_data(0x3A)
        H1_T0_OUT_msb = self._read_data(0x3B)
        H1_T0_OUT = float(_to_signed((H1_T0_OUT_msb << 8) + H1_T0_OUT_lsb))

        T0_degC_x8_lsb = self._read_data(0x32)
        T1_degC_x8_lsb = self._read_data(0x33)
        T1_T0_degC_x8_msb = self._read_data(0x35)
        T0_degC_x8 = ((T1_T0_degC_x8_msb & 0x03) << 8) + T0_degC_x8_lsb
        T0_degC = T0_degC_x8 / 8.0
        T1_degC_x8 = ((T1_T0_degC_x8_msb & (0x03 << 2)) << 6) + T1_degC_x8_lsb
        T1_degC = T1_degC_x8 / 8.0

        T0_OUT_lsb = self._read_data(0x3C)
        T0_OUT_msb = self._read_data(0x3D)
        T0_OUT = float(_to_signed((T0_OUT_msb << 8) + T0_OUT_lsb))
        T1_OUT_lsb = self._read_data(0x3E)
        T1_OUT_msb = self._read_data(0x3F)
        T1_OUT = float(_to_signed((T1_OUT_msb << 8) + T1_OUT_lsb))

        self.a_t = (T1_degC - T0_degC) / (T1_OUT - T0_OUT)
        self.b_t = T0_degC - self.a_t * T0_OUT
        self.a_h = (H1_rH - H0_rH) / (H1_T0_OUT - H0_T0_OUT)
        self.b_h = H0_rH - self.a_h * H0_T0_OUT

    def read_temperature_and_humidity(self):
        # humidity msb, humidity lsb, temp msb, temp lsb
        # Read data back from 0x28(40) with command register 0x80(128)
        data = self._bus.read_i2c_block_data(self._hts221_address, 0x28 | 0x80, 4)

        # noinspection PyPep8Naming
        H_OUT = _to_signed(data[1] << 8) + data[0]
        # noinspection PyPep8Naming
        T_OUT = _to_signed(data[3] << 8) + data[2]

        temperature = self.a_t * T_OUT + self.b_t
        humidity = self.a_h * H_OUT + self.b_h

        return temperature, humidity


def _main():
    sensor = Hts221()
    temperature, humidity = sensor.read_temperature_and_humidity()
    print "Temperature: %.2f C" % temperature
    print "Relative Humidity : %.2f %%" % humidity


if __name__ == '__main__':
    _main()
