#!/usr/bin/env python3

# Micro Python library that brings out-of-the-box Blynk support to
# the WiPy. Requires a previously established internet connection
# and a valid token string.
# 
# Example usage:
# 
#     import BlynkLib
#     import time
# 
#     blynk = BlynkLib.Blynk('08a46fbc7f57407995f576f3f84c3f72')
# 
#     # define a virtual pin read handler
#     def v0_read_handler():
#         # we must call virtual write in order to send the value to the widget
#         blynk.virtual_write(0, time.ticks_ms() // 1000)
# 
#     # register the virtual pin
#     blynk.add_virtual_pin(0, read=v0_read_handler)
# 
#     # define a virtual pin write handler
#     def v1_write_handler(value):
#         print(value)
# 
#     # register the virtual pin
#     blynk.add_virtual_pin(1, write=v1_write_handler)
# 
#     # register the task running every 3 sec
#     # (period must be a multiple of 50 ms)
#     def my_user_task():
#         # do any non-blocking operations
#         print('Action')
# 
#     blynk.set_user_task(my_user_task, 3000)
# 
#     # start Blynk (this call should never return)
#     blynk.run()
# 
# -----------------------------------------------------------------------------
# 
# This file is part of the Micro Python project, http://micropython.org/
# 
# The MIT License (MIT)
# 
# Copyright (c) 2015 Daniel Campora
# Copyright (c) 2015 Volodymyr Shymanskyy
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import socket
import struct
import time
import os
from enum import Enum

const = lambda x: x

HDR_LEN = const(5)
HDR_FMT = "!BHH"

MAX_MSG_PER_SEC = const(20)

MSG_RSP = const(0)
MSG_LOGIN = const(2)
MSG_PING = const(6)
MSG_TWEET = const(12)
MSG_EMAIL = const(13)
MSG_NOTIFY = const(14)
MSG_BRIDGE = const(15)
MSG_HW_SYNC = const(16)
MSG_HW_INFO = const(17)
MSG_HW = const(20)


class Msg(Enum):
    RSP = 0
    LOGIN = 2
    PING = 6
    TWEET = 12
    EMAIL = 13
    NOTIFY = 14
    BRIDGE = 15
    HW_SYNC = 16
    HW_INFO = 17
    HW = 20


STA_SUCCESS = const(200)

HB_PERIOD = const(10)
NON_BLK_SOCK = const(0)
MIN_SOCK_TO = const(1)  # 1 second
MAX_SOCK_TO = const(5)  # 5 seconds, must be < HB_PERIOD
WDT_TO = const(10000)  # 10 seconds
RECONNECT_DELAY = const(1)  # 1 second
TASK_PERIOD_RES = const(50)  # 50 ms
IDLE_TIME_MS = const(5)  # 5 ms

RE_TX_DELAY = const(2)
MAX_TX_RETRIES = const(3)
MAX_RX_RETRIES = const(3)

MAX_VIRTUAL_PINS = const(32)

DISCONNECTED = 0
CONNECTING = 1
AUTHENTICATING = 2
AUTHENTICATED = 3

EAGAIN = const(11)

log = logging.getLogger('blynk')
logging.basicConfig(level=logging.INFO)


class Pin(object):
    def __init__(self, pin_no):
        """
        :param int pin_no:
        """
        self._pin_no = pin_no

    def read(self):
        """
        :rtype int:
        """
        raise NotImplementedError()

    def write(self, value):
        """
        :param int value:
        """
        raise NotImplementedError()


class VrPin(Pin):
    def __init__(self, pin_no, blynk_client, read=None, write=None):
        super(VrPin, self).__init__(pin_no)
        self._read = read
        self._write = write
        self._blynk_client = blynk_client

    def read(self):
        if self._read is None:
            return self._read(self._pin_no, self._blynk_client)
        else:
            raise IOError('Unable to read from VrPin {}.'.format(self._pin_no))

    def write(self, value):
        if self._read is None:
            self._write(value, self._pin_no, self._blynk_client)
        else:
            raise IOError('Unable to write to VrPin {}'.format(self._pin_no))


class HwPin(Pin):
    IN = 'in'
    OUT = 'out'
    """
    Specific board implementation to be created based on this abstraction
    digital-analog and in-out mapping is obtain from board.json
    """

    def __init__(self, pin_no, mode):
        # TODO read json for pins capabilities
        """
        :param in pin_no:
        :param HwPin.IN|HwPin.OUT mode:
        """
        super(HwPin, self).__init__(pin_no)
        self._mode = mode
        self._configure()

    def read(self):
        raise NotImplementedError()

    def write(self, value):
        raise NotImplementedError()

    def _configure(self):
        raise NotImplementedError()


def nop(*args, **kwargs):
    pass


def not_implemented(*args, **kwargs):
    raise NotImplementedError()


class BlynkConnectionError(Exception):
    pass


class NoBlynkData(Exception):
    pass


class BlynkConnection(object):
    def __init__(self, token, server='blynk-cloud.com', port=None, ssl=False):
        self._last_tx_time = 0
        self._do_connect = True
        self._token = token
        self._server = server
        if port is None:
            if ssl:
                port = 8441
            else:
                port = 8442
        self._port = port
        self._ssl = ssl
        self.state = DISCONNECTED

        self._on_connect = nop
        self._on_disconnect = nop
        self._last_hb_time = 0
        self._last_recv_time = 0
        self._handle_hw_message = not_implemented

        self._clear_recv_data()

    def set_handle_hw_message(self, func):
        self._handle_hw_message = func

    def call_on_connect(self, func):
        self._on_connect = func

    def call_on_disconnect(self, func):
        self._on_disconnect = func

    def _new_msg_id(self):
        self._msg_id += 1
        if self._msg_id > 0xFFFF:
            self._msg_id = 1
        return self._msg_id

    def _clear_recv_data(self):
        self._rx_data = b''
        self._pending_header = None

    def _connect_and_authenticate(self):
        self._clear_recv_data()
        try:
            self.state = CONNECTING
            if self._ssl:
                import ssl
                log.info('SSL: Connecting to %s:%d' % (self._server, self._port))
                ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_SEC)
                self.conn = ssl.wrap_socket(ss, cert_reqs=ssl.CERT_REQUIRED, ca_certs='/flash/cert/ca.pem')
            else:
                log.info('TCP: Connecting to %s:%d' % (self._server, self._port))
                self.conn = socket.socket()
            self.conn.connect(socket.getaddrinfo(self._server, self._port)[0][4])
        except socket.error as ex:
            self._close()
            raise BlynkConnectionError('Connection with the Blynk servers failed: {}'.format(ex))

        self.state = AUTHENTICATING
        log.info('Blynk connection successful, authenticating...')
        self._send_message(Msg.LOGIN, self._new_msg_id(), self._token)  # dsidor
        try:
            msg_type, msg_id, msg_len, data = self._receive_message(MAX_SOCK_TO)
        except NoBlynkData:
            self._close()
            raise BlynkConnectionError('Blynk authentication timed out')
        if data != STA_SUCCESS or msg_id == 0:
            self._close()
            raise BlynkConnectionError('Blynk authentication failed: status: {}, msg_id: {}'.format(data, msg_id))

        self.state = AUTHENTICATED
        # dsidor board info etc
        self._send_message(Msg.HW_INFO, self._new_msg_id(), "h-beat", HB_PERIOD, 'dev', 'WiPy', "cpu", "CC3200")
        log.info('Blynk authenticated')
        if self._on_connect:
            self._on_connect()

    def _close(self):
        self.conn.close()
        self.state = DISCONNECTED

    def _recv(self, length, timeout=0):
        self.conn.settimeout(timeout)
        for _ in range(0, MAX_RX_RETRIES):
            try:
                self._rx_data += self.conn.recv(length)
                break
            except socket.timeout:
                raise NoBlynkData()
            except socket.error as e:
                if e.args[0] == EAGAIN:
                    continue
                else:
                    raise
        if len(self._rx_data) >= length:
            data = self._rx_data[:length]
            self._rx_data = self._rx_data[length:]
            return data
        else:
            raise NoBlynkData()

    def _receive_message(self, timeout):
        if self._pending_header is None:
            data = self._recv(HDR_LEN, timeout)
            try:
                msg_type, msg_id, msg_len = struct.unpack(HDR_FMT, data)
            except struct.error:
                raise BlynkConnectionError('Unable to parse received message: (hex){}'.format(data.encode('hex')))
        else:
            msg_type, msg_id, msg_len = self._pending_header

        if msg_type == MSG_RSP:
            # In MSG_RSP there is no data - status is in msg_len field...
            msg_data = msg_len
            msg_len = 2
        else:
            msg_data = b''
            if msg_len > 0:
                try:
                    msg_data = self._recv(msg_len, 0)
                except NoBlynkData:
                    self._pending_header = msg_type, msg_id, msg_len
                    raise
                self._pending_header = None

        msg_type = Msg(msg_type)
        log.debug('Received: msg_type: {}, msg_id: {}, msg_len: {}, msg_data: "{}"'.format(msg_type.name, msg_id,
                                                                                           msg_len, msg_data))
        self._last_recv_time = time.time()
        return msg_type, msg_id, msg_len, msg_data

    def _handle_message(self, message):
        msg_type, msg_id, msg_len, msg_data = message
        if msg_type == Msg.RSP:
            log.debug('MSG_RSP received')
            pass
        elif msg_type == Msg.PING:
            log.error('got ping message - server sends it sometimes, delete this helper')
            self._send_message(Msg.RSP, msg_id, STA_SUCCESS)
        elif msg_type == Msg.HW or msg_type == Msg.BRIDGE:
            if msg_data:
                self._handle_hw_message(msg_data)
        else:
            raise BlynkConnectionError('Unsupported message type received: {}'.format(msg_type))

    def _send_message(self, msg_type, msg_id, *msg_datas):
        """
        :param Msg msg_type:
        :param int msg_id:
        :param msg_datas:
        :return:
        """
        data = bytes('\0'.join(map(str, msg_datas)), 'ascii')
        log.debug('Sending: msg_type: {}, msg_id: {}, msg_len: {}, msg_data: {}'.format(msg_type.name, msg_id,
                                                                                        len(data), data))
        packed = struct.pack(HDR_FMT, msg_type.value, msg_id, len(data)) + data
        self._send(packed)

    def send_message(self, msg_type, *msg_datas):
        self._send_message(msg_type, self._new_msg_id(), *msg_datas)

    def _send(self, data):
        retries = 0
        tx_delay = self._last_tx_time + 1 / MAX_VIRTUAL_PINS - time.time()
        if tx_delay > 0:
            log.info('Sleeping before send for {}s'.format(tx_delay))
            time.sleep(tx_delay)
        while retries <= MAX_TX_RETRIES:
            try:
                self.conn.send(data)
                break
            except socket.error as er:
                if er.args[0] != EAGAIN:
                    raise
                else:
                    time.sleep(RE_TX_DELAY)
                    retries += 1
        self._last_tx_time = time.time()

    def _server_alive(self):
        now = time.time()
        if now - self._last_recv_time > 2 * HB_PERIOD:
            return False
        else:
            return True

    def _heart_beat(self):
        now = time.time()
        if now - self._last_hb_time >= HB_PERIOD and self.state == AUTHENTICATED:
            self._send_message(Msg.PING, self._new_msg_id(), '')
            self._last_hb_time = now

    def run(self):
        self.state = DISCONNECTED
        now = time.time()
        self._last_hb_time = now
        self._last_recv_time = now
        self._msg_id = 1
        while True:
            while self._do_connect and self.state != AUTHENTICATED:
                try:
                    self._connect_and_authenticate()
                except BlynkConnectionError as ex:
                    log.error(str(ex))
                    time.sleep(RECONNECT_DELAY)

            while self._do_connect:
                try:
                    message = self._receive_message(timeout=MIN_SOCK_TO)
                    self._handle_message(message)
                except NoBlynkData:
                    pass
                except BlynkConnectionError as ex:
                    log.error(str(ex))
                    self._close()
                    break

                self._heart_beat()
                if not self._server_alive():
                    self._close()
                    log.error('Blynk server is offline')
                    break
            else:
                self._close()
                log.info('Blynk disconnection requested by the user')
                return


class BlynkClient(object):
    def __init__(self, blynk_connection):
        """
        :param BlynkConnection blynk_connection:
        """
        self._blynk_conn = blynk_connection
        self._vr_pins = {}
        self._hw_pins = {}

        self._blynk_conn.set_handle_hw_message(self._handle_hw_message)

    def run(self):
        self._blynk_conn.run()

    def add_virtual_pin(self, pin, read=None, write=None):
        """
        :param int pin:
        :param read:
        :param write:
        :return:
        """
        if pin < 0 or pin > MAX_VIRTUAL_PINS:
            raise ValueError('Virtual pin should be in range between {} and {}'.format(0, MAX_VIRTUAL_PINS - 1))
        # TODO can I assert function prototype?
        log.info('Registered {} virtual pin'.format(pin))
        self._vr_pins[pin] = VrPin(pin, self, read, write)

    def notify(self, msg):
        self._blynk_conn.send_message(Msg.NOTIFY, msg)

    def tweet(self, msg):
        self._blynk_conn.send_message(Msg.NOTIFY, msg)

    def email(self, to, subject, body):
        self._blynk_conn.send_message(Msg.EMAIL, to, subject, body)

    def virtual_write(self, pin, val):
        self._blynk_conn.send_message(Msg.HW, 'vw', pin, val)

    def sync_all(self):
        # TODO what it does?
        self._blynk_conn.send_message(Msg.HW_SYNC)

    def sync_virtual(self, pin):
        # TODO what it does?
        self._blynk_conn.send_message(Msg.HW_SYNC, 'vr', pin)

        # TODO get to know what it is
        # def repl(self, pin):
        #     repl = Terminal(self, pin)
        #     self.add_virtual_pin(pin, repl.virtual_read, repl.virtual_write)
        #     return repl

    def _handle_hw_message(self, data):
        params = list(map(lambda x: x.decode('ascii'), data.split(b'\0')))
        cmd = params.pop(0)
        commands = {
            'info': self._handle_info,
            'pm': self._handle_pin_mode,
            'vr': self._handle_virtual_read,
            'vw': self._handle_virtual_write,
            'dr': self._handle_digital_read,
            'dw': self._handle_digital_write,
            'ar': self._handle_analog_read,
            'aw': self._handle_analog_write,
        }
        if cmd in commands:
            commands[cmd](params)
        else:
            log.error('Unsupported command {}'.format(cmd))

    def _handle_info(self, params):
        pass

    def _handle_pin_mode(self, params):
        # TODO to implement
        log.error('Not implemented _handle_pin_mode, got params: {}'.format(params))
        # pairs = zip(params[0::2], params[1::2])
        # for (pin, mode) in pairs:
        #     pin = int(pin)
        #     if mode != 'in' and mode != 'out' and mode != 'pu' and mode != 'pd':
        #         raise ValueError("Unknown pin %d mode: %s" % (pin, mode))
        #     self._hw_pins[pin] = HwPin(pin, mode, mode)
        # self._pins_configured = True

    def _handle_virtual_read(self, params):
        pin = int(params.pop(0))
        if pin not in self._vr_pins:
            log.warning('Virtual read of unregistered pin {}'.format(pin))
        else:
            self.virtual_write(pin, self._vr_pins[pin].read())

    def _handle_virtual_write(self, params):
        pin = int(params.pop(0))
        if pin not in self._vr_pins:
            log.warning('Virtual write to unregistered pin {}'.format(pin))
        else:
            # TODO should write in for or all list at once?
            for param in params:
                self._vr_pins[pin].write(param)

    def _handle_digital_read(self, params):
        pin = int(params.pop(0))
        if pin in self._hw_pins:
            val = self._hw_pins[pin].digital_read()
            self._blynk_conn.send_message(Msg.HW, 'dw', pin, val)
        else:
            log.warning('Digital read of not configured pin {}'.format(pin))

    def _handle_digital_write(self, params):
        pin = int(params.pop(0))
        val = int(params.pop(0))
        if pin in self._hw_pins:
            self._hw_pins[pin].digital_write(val)
        else:
            log.warning('Digital write to not configured pin {}'.format(pin))

    def _handle_analog_read(self, params):
        pin = int(params.pop(0))
        if pin in self._hw_pins:
            val = self._hw_pins[pin].analog_read()
            self._blynk_conn.send_message(Msg.HW, 'aw', pin, val)
        else:
            log.warning('Digital write to not configured pin {}'.format(pin))

    def _handle_analog_write(self, params):
        pin = int(params.pop(0))
        val = int(params.pop(0))
        if pin in self._hw_pins:
            self._hw_pins[pin].analog_write(val)
        else:
            log.warning('Analog write to not configured pin {}'.format(pin))
