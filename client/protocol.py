""" protocol.py

    Author:     Mike Pagel <https://github.com/moltob>
    Date:       11/01/2016
    Modified:
        03/18/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Adapted for IDE virtual NXP Cup.
"""
import enum
import logging
import socket

from client.car import Actuator, Sensor
from client.driver import Driver

_logger = logging.getLogger(__name__)

MSG_IDENTIFIED = b'***identified***'
MSG_SHUTDOWN = b'***shutdown***'
MSG_RESTART = b'***restart***'

TO_SOCKET_SEC = 1
TO_SOCKET_MSEC = TO_SOCKET_SEC * 1000

RNG_FNDR_ANGLES = [-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90]


class State(enum.Enum):
    """ Enumeration representing the current state of the connection

        STOPPED     - Not connected to the TORCS server
        STARTING    - Entering cyclic communication
        RUNNING     - Connected to TORCS server; Evaluating driver logic
        STOPPING    - Exiting cyclic communication
    """
    STOPPED = 1
    STARTING = 2
    RUNNING = 3
    STOPPING = 4


class Client:
    """ Client for TORCS racing car simulation with SCRC network server.

        Attributes:
            hostaddr        - Tuple of (hostname, port) for the TORCS server.
            driver          - Driving logic implementation.
            state (State)   - Runtime state of the client.
            socket (socket) - UDP socket to server.
    """

    def __init__(self, host, port):
        self.hostaddr = (host, port)
        self.state = State.STOPPED
        self.socket = None
        self.driver = Driver()

    def run(self):
        """ Enters the cyclic execution of the client network interface.
        """
        _logger.info('[Client] Execution starting...')
        if self.state is State.STOPPED:
            self._start()

        while self.state is State.RUNNING:
            self._process()

        _logger.info('[Client] Execution stopped.')
        self.state = State.STOPPED

    def _start(self):
        _logger.info('[Client] \tInitializing connection...')
        self.state = State.STARTING

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(TO_SOCKET_SEC)

        try:
            self._register()
            self.state = State.RUNNING
        except socket.error as err:
            _logger.error('[Client] \tFailed to initialize connection: {}'.format(err))
            self.state = State.STOPPED

    def _stop(self):
        if self.state is State.RUNNING:
            _logger.info('[Client] \tDisconnecting from server...')
            self.state = State.STOPPING

    def _process(self):
        try:
            buffer, _ = self.socket.recvfrom(TO_SOCKET_MSEC)
            if not buffer:
                return
            elif MSG_SHUTDOWN in buffer:
                _logger.info('[Client] \tServer requesting shutdown...')
                self._stop()
            elif MSG_RESTART in buffer:
                _logger.info('[Client] \tServer requesting restart...')
            else:
                sensor_data = Serializer.decode(buffer)
                sensor = Sensor(sensor_data)

                command = self.driver.drive(sensor)

                buffer = Serializer.encode(command.actuator_dict)
                self.socket.sendto(buffer, self.hostaddr)

        except socket.error as err:
            _logger.warning('[Client] \tCommunication with server failed: {}.'.format(err))

        except KeyboardInterrupt:
            _logger.info('[Client] \tRecieved keyboard interrupt...')
            self._stop()

    def _register(self):
        data = {'init': RNG_FNDR_ANGLES}
        buffer = Serializer.encode(data, prefix='SCR-{}'.format(self.hostaddr[1]))

        connected = False
        while not connected and self.state is not State.STOPPING:
            try:
                self.socket.sendto(buffer, self.hostaddr)

                buffer, _ = self.socket.recvfrom(TO_SOCKET_MSEC)
                if MSG_IDENTIFIED in buffer:
                    connected = True

            except socket.error as err:
                _logger.debug('No connection to server yet: {}'.format(err))


class Serializer():
    """ Serializes data for communication between TORCS client and server """

    @staticmethod
    def encode(data, prefix=None):
        """ Encodes data from a given dictionary for transmission between a
            TORCS server and client

            Args:
                data    - Dictionary of elements to encode.
                prefix  - Optional prefix string

            Returns: The encoded data as a series of bytes
        """

        elements = []
        if prefix:
            elements.append(prefix)

        for k, v in data.items():
            if v and v[0] is not None:
                vstr = map(lambda i: str(i), v)
                elements.append('({} {})'.format(k, ' '.join(vstr)))

        return ''.join(elements).encode()

    @staticmethod
    def decode(data):
        """ Decodes response from server and returns a dictionary of data

            Args:
                data    - Series of bytes to decode.

            Returns: The decoded data as a python dictionary
        """
        d = {}
        s = data.decode()

        pos = 0
        while len(s) > pos:
            start = s.find('(', pos)
            if start < 0:
                # end of list:
                break

            end = s.find(')', start + 1)
            if end < 0:
                _logger.warning('Opening brace at position {} not matched in '
                                'buffer {!r}.'.format(start, buffer))
                break

            items = s[start + 1:end].split(' ')
            if len(items) < 2:
                _logger.warning('Buffer {!r} not holding proper key value pair.'.format(buffer))
            else:
                key = items[0]
                if len(items) == 2:
                    value = items[1]
                else:
                    value = items[1:]
                d[key] = value

            pos = end + 1

        return d
