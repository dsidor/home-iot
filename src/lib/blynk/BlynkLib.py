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
    def __init__(self, pin_no, read=None, write=None):
        super(VrPin, self).__init__(pin_no)
        self._read = read
        self._write = write

    def read(self):
        if self._read is None:
            return self._read()
        else:
            raise IOError('Unable to read from VrPin {}.'.format(self._pin_no))

    def write(self, value):
        if self._read is None:
            self._write(value)
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


class BlynkConnectionError(Exception):
    pass


class NoBlynkData(Exception):
    pass


class BlynkConnection(object):
    def __init__(self, token, server='blynk-cloud.com', port=None, ssl=False):
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

        def nop():
            pass

        self._on_connect = nop
        self._on_disconnect = nop
        self._last_hb_time = 0
        self._last_recv_time = 0

        self._clear_recv_data()

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
        self._last_recv_time = int(time.time())
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
                return
                self._handle_hw(msg_data)
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

    def _send(self, data):
        retries = 0
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

    def _server_alive(self):
        now = int(time.time())
        if now - self._last_recv_time > 2 * HB_PERIOD:
            return False
        else:
            return True

    def _heart_beat(self):
        now = int(time.time())
        if now - self._last_hb_time >= HB_PERIOD and self.state == AUTHENTICATED:
            self._send_message(Msg.PING, self._new_msg_id(), '')
            self._last_hb_time = now

    def run(self):
        self.state = DISCONNECTED
        now = int(time.time())
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
    def __init__(self):
        self._vr_pins = {}
        self._hw_pins = {}


class Blynk(object):
    def __init__(self, token, server='blynk-cloud.com', port=None, connect=True, kick_watchdog=None, ssl=False):
        self._token = token
        if isinstance(self._token, str):
            self._token = bytes(token, 'ascii')
        self._server = server
        self._do_connect = connect
        if kick_watchdog is None:
            def nop():
                pass

            kick_watchdog = nop
        self._kick_watchdog = kick_watchdog
        if port is None:
            if ssl:
                port = 8441
            else:
                port = 8442
        self._port = port
        self._ssl = ssl

        self._vr_pins = {}
        self._hw_pins = {}
        self.state = DISCONNECTED
        self._on_connect = None

    def _format_msg(self, msg_type, *args):
        data = bytes('\0'.join(map(str, args)), 'ascii')
        return struct.pack(HDR_FMT, msg_type, self._new_msg_id(), len(data)) + data

    def _handle_hw(self, data):
        params = list(map(lambda x: x.decode('ascii'), data.split(b'\0')))
        cmd = params.pop(0)
        if cmd == 'info':
            pass
        elif cmd == 'pm':
            pairs = zip(params[0::2], params[1::2])
            for (pin, mode) in pairs:
                pin = int(pin)
                if mode != 'in' and mode != 'out' and mode != 'pu' and mode != 'pd':
                    raise ValueError("Unknown pin %d mode: %s" % (pin, mode))
                self._hw_pins[pin] = HwPin(pin, mode, mode)
            self._pins_configured = True
        elif cmd == 'vw':
            pin = int(params.pop(0))
            if pin in self._vr_pins and self._vr_pins[pin].write:
                for param in params:
                    self._vr_pins[pin].write(param)
            else:
                print("Warning: Virtual write to unregistered pin %d" % pin)
        elif cmd == 'vr':
            pin = int(params.pop(0))
            if pin in self._vr_pins and self._vr_pins[pin].read:
                self._vr_pins[pin].read()
            else:
                print("Warning: Virtual read from unregistered pin %d" % pin)
        elif self._pins_configured:
            if cmd == 'dw':
                pin = int(params.pop(0))
                val = int(params.pop(0))
                self._hw_pins[pin].digital_write(val)
            elif cmd == 'aw':
                pin = int(params.pop(0))
                val = int(params.pop(0))
                self._hw_pins[pin].analog_write(val)
            elif cmd == 'dr':
                pin = int(params.pop(0))
                val = self._hw_pins[pin].digital_read()
                self._send(self._format_msg(MSG_HW, 'dw', pin, val))
            elif cmd == 'ar':
                pin = int(params.pop(0))
                val = self._hw_pins[pin].analog_read()
                self._send(self._format_msg(MSG_HW, 'aw', pin, val))
            else:
                raise ValueError("Unknown message cmd: %s" % cmd)

    def _new_msg_id(self):
        self._msg_id += 1
        if (self._msg_id > 0xFFFF):
            self._msg_id = 1
        return self._msg_id

    def _settimeout(self, timeout):
        if timeout != self._timeout:
            self._timeout = timeout
            self.conn.settimeout(timeout)

    def _recv(self, length, timeout=0):
        self._settimeout(timeout)
        try:
            self._rx_data += self.conn.recv(length)
        except socket.timeout:
            return b''
        except socket.error as e:
            if e.args[0] == EAGAIN:
                return b''
            else:
                raise
        if len(self._rx_data) >= length:
            data = self._rx_data[:length]
            self._rx_data = self._rx_data[length:]
            return data
        else:
            return b''

    def _send(self, data, send_anyway=False):
        if self._tx_count < MAX_MSG_PER_SEC or send_anyway:
            retries = 0
            while retries <= MAX_TX_RETRIES:
                try:
                    self.conn.send(data)
                    self._tx_count += 1
                    break
                except socket.error as er:
                    if er.args[0] != EAGAIN:
                        raise
                    else:
                        time.sleep_ms(RE_TX_DELAY)
                        retries += 1

    def _close(self, emsg=None):
        self.conn.close()
        self.state = DISCONNECTED
        time.sleep(RECONNECT_DELAY)
        if emsg:
            print('Error: %s, connection closed' % emsg)

    def _server_alive(self):
        c_time = int(time.time())
        if self._m_time != c_time:
            self._m_time = c_time
            self._tx_count = 0
            if self._last_hb_id != 0 and c_time - self._hb_time >= MAX_SOCK_TO:
                return False
            if c_time - self._hb_time >= HB_PERIOD and self.state == AUTHENTICATED:
                self._hb_time = c_time
                self._last_hb_id = self._new_msg_id()
                self._send(struct.pack(HDR_FMT, MSG_PING, self._last_hb_id, 0), True)
        return True

    def repl(self, pin):
        repl = Terminal(self, pin)
        self.add_virtual_pin(pin, repl.virtual_read, repl.virtual_write)
        return repl

    def notify(self, msg):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_NOTIFY, msg))

    def tweet(self, msg):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_TWEET, msg))

    def email(self, to, subject, body):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_EMAIL, to, subject, body))

    def virtual_write(self, pin, val):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_HW, 'vw', pin, val))

    def sync_all(self):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_HW_SYNC))

    def sync_virtual(self, pin):
        if self.state == AUTHENTICATED:
            self._send(self._format_msg(MSG_HW_SYNC, 'vr', pin))

    def add_virtual_pin(self, pin, read=None, write=None):
        if isinstance(pin, int) and pin in range(0, MAX_VIRTUAL_PINS):
            self._vr_pins[pin] = VrPin(read, write)
        else:
            raise ValueError('the pin must be an integer between 0 and %d' % (MAX_VIRTUAL_PINS - 1))

    def call_on_connect(self, func):
        self._on_connect = func

    def set_user_tasks(self):
        # TODO
        raise NotImplementedError()

    def connect(self):
        self._do_connect = True

    def disconnect(self):
        self._do_connect = False

    def run(self):
        self._start_time = time.ticks_ms()
        self._hw_pins = {}
        self._rx_data = b''
        self._msg_id = 1
        self._pins_configured = False
        self._timeout = None
        self._tx_count = 0
        self._m_time = 0
        self.state = DISCONNECTED

        while True:
            while self.state != AUTHENTICATED:
                if self._do_connect:
                    try:
                        self.state = CONNECTING
                        if self._ssl:
                            import ssl
                            print('SSL: Connecting to %s:%d' % (self._server, self._port))
                            ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_SEC)
                            self.conn = ssl.wrap_socket(ss, cert_reqs=ssl.CERT_REQUIRED, ca_certs='/flash/cert/ca.pem')
                        else:
                            print('TCP: Connecting to %s:%d' % (self._server, self._port))
                            self.conn = socket.socket()
                        self.conn.connect(socket.getaddrinfo(self._server, self._port)[0][4])
                    except:
                        self._close('connection with the Blynk servers failed')
                        continue

                    self.state = AUTHENTICATING
                    hdr = struct.pack(HDR_FMT, MSG_LOGIN, self._new_msg_id(), len(self._token))  # dsidor
                    print('Blynk connection successful, authenticating...')
                    self._send(hdr + self._token, True)
                    data = self._recv(HDR_LEN, timeout=MAX_SOCK_TO)
                    if not data:
                        self._close('Blynk authentication timed out')
                        continue

                    msg_type, msg_id, status = struct.unpack(HDR_FMT, data)
                    if status != STA_SUCCESS or msg_id == 0:
                        self._close('Blynk authentication failed')
                        continue

                    self.state = AUTHENTICATED
                    self._send(self._format_msg(MSG_HW_INFO, "h-beat", HB_PERIOD, 'dev', 'WiPy', "cpu", "CC3200"))
                    print('Access granted, happy Blynking!')
                    if self._on_connect:
                        self._on_connect()
                else:
                    self._start_time = sleep_from_until(self._start_time, TASK_PERIOD_RES)

            self._hb_time = 0
            self._last_hb_id = 0
            self._tx_count = 0
            while self._do_connect:
                data = self._recv(HDR_LEN, NON_BLK_SOCK)
                if data:
                    msg_type, msg_id, msg_len = struct.unpack(HDR_FMT, data)
                    if msg_id == 0:
                        self._close('invalid msg id %d' % msg_id)
                        break
                    if msg_type == MSG_RSP:
                        if msg_id == self._last_hb_id:
                            self._last_hb_id = 0
                    elif msg_type == MSG_PING:
                        self._send(struct.pack(HDR_FMT, MSG_RSP, msg_id, STA_SUCCESS), True)
                    elif msg_type == MSG_HW or msg_type == MSG_BRIDGE:
                        data = self._recv(msg_len, MIN_SOCK_TO)
                        if data:
                            self._handle_hw(data)
                    else:
                        self._close('unknown message type %d' % msg_type)
                        break
                else:
                    self._start_time = sleep_from_until(self._start_time, IDLE_TIME_MS)
                if not self._server_alive():
                    self._close('Blynk server is offline')
                    break

            if not self._do_connect:
                self._close()
                print('Blynk disconnection requested by the user')
