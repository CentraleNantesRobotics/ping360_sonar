from brping import definitions, PingMessage, PingParser
import time
import errno
import math
import numpy as np
import random
import perlin

verbose = False
payload_dict = definitions.payload_dict_all

# A Serial class emulator
# Author: D. Thiebaut
# This program energizes the fakeSerial simulator using example code taken
# from http://pyserial.sourceforge.net/shortintro.html


class Serial:

    # init(): the constructor.  Many of the arguments have default values
    # and can be skipped when calling the constructor.
    def __init__(self, port='COM1', baudrate=115200, timeout=1,
                 bytesize=8, parity='N', stopbits=1, xonxoff=0,
                 rtscts=0):
        self.name = port
        self.port = port
        self.timeout = timeout
        self.parity = parity
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.stopbits = stopbits
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self._isOpen = True
        self._receivedData = ""
        self.in_waiting = 1

        self.parser = PingParser()  # used to parse incoming client comunications

        self._gain_setting = 0
        self._mode = 0
        self._angle = 0
        self._transmit_duration = 0
        self._sample_period = 0
        self._transmit_frequency = 100
        self._number_of_samples = 10
        self._data = "".join([chr(0) for _ in xrange(self._number_of_samples)])
        self._data_length = 10

        self._noise = perlin.noise(400, 50, 50)

    # isOpen()
    # returns True if the port to the Arduino is open.  False otherwise
    def isOpen(self):
        return self._isOpen

    def send_break(self):
        pass

    # open()
    # opens the port
    def open(self):
        self._isOpen = True

    # close()
    # closes the port
    def close(self):
        self._isOpen = False

    # write()
    # writes a string of characters to the Arduino
    def write(self, data):
        try:
            # digest data coming in from client
            for byte in data:
                if self.parser.parse_byte(byte) == PingParser.NEW_MESSAGE:
                    # we decoded a message from the client
                    self.handleMessage(self.parser.rx_msg)

        except EnvironmentError as e:
            if e.errno == errno.EAGAIN:
                pass  # waiting for data
            else:
                if verbose:
                    print("Error reading data", e)

        except KeyError as e:
            if verbose:
                print("skipping unrecognized message id: %d" %
                      self.parser.rx_msg.message_id)
                print("contents: %s" % self.parser.rx_msg.msg_data)
            pass

    # read()
    # reads n characters from the fake Arduino. Actually n characters
    # are read from the string _data and returned to the caller.
    def read(self, n=1):
        s = self._read_data[0:n]
        self._read_data = self._read_data[n:]
        return s

    # readline()
    # reads characters from the fake Arduino until a \n is found.
    def readline(self):
        returnIndex = self._read_data.index("\n")
        if returnIndex != -1:
            s = self._read_data[0:returnIndex + 1]
            self._read_data = self._read_data[returnIndex + 1:]
            return s
        else:
            return ""

    # __str__()
    # returns a string representation of the serial class
    def __str__(self):
        return "Serial<id=0xa81c10, open=%s>( port='%s', baudrate=%d," \
            % (str(self.isOpen), self.port, self.baudrate) \
            + " bytesize=%d, parity='%s', stopbits=%d, xonxoff=%d, rtscts=%d)" \
            % (self.bytesize, self.parity, self.stopbits, self.xonxoff,
               self.rtscts)

    # Send a message to the client, the message fields are populated by the
    # attributes of this object (either variable or method) with names matching
    # the message field names
    def sendMessage(self, message_id):
        msg = PingMessage(message_id)
        if verbose:
            print("sending message %d\t(%s)" % (msg.message_id, msg.name))
        # pull attributes of this class into the message fields (they are named the same)
        for attr in payload_dict[message_id]["field_names"]:
            try:
                # see if we have a function for this attribute (dynamic data)
                # if so, call it and put the result in the message field
                setattr(msg, attr, getattr(self, attr)())
            except AttributeError as e:
                try:
                    # if we don't have a function for this attribute, check for a _<field_name> member
                    # these are static values (or no function implemented yet)
                    setattr(msg, attr, getattr(self, "_" + attr))
                except AttributeError as e:
                    # anything else we haven't implemented yet, just send a sine wave
                    setattr(msg, attr, self.periodicFnInt(20, 120))
        # send the message to the client
        msg.pack_msg_data()
        self._read_data = msg.msg_data

    # handle an incoming client message
    def handleMessage(self, message):
        if verbose:
            print("receive message %d\t(%s)" %
                  (message.message_id, message.name))
        if message.message_id == definitions.COMMON_GENERAL_REQUEST:
            # the client is requesting a message from us
            self.sendMessage(message.requested_id)
        # hack for legacy requests
        elif message.payload_length == 0:
            self.sendMessage(message.message_id)
        elif message.message_id == definitions.PING360_TRANSDUCER:
            # the client is controlling some parameter of the device
            self.setParameters(message)
        else:
            if verbose:
                print("Unknown msg type")

    # Extract message fields into attribute values
    # This should only be performed with the 'set' category of messages
    # TODO: mechanism to filter by "set"
    def setParameters(self, message):
        for attr in payload_dict[message.message_id]["field_names"]:
            setattr(self, "_" + attr, getattr(message, attr))
        self.sendDataResponse(message)

    def sendDataResponse(self, message):
        # Send a response
        self.generateRandomData()
        msg = PingMessage(definitions.PING360_DEVICE_DATA)
        if verbose:
            print("sending a reply %d\t(%s)" % (msg.message_id, msg.name))
        # pull attributes of this class into the message fields (they are named the same)
        for attr in payload_dict[definitions.PING360_DEVICE_DATA]["field_names"]:
            setattr(msg, attr, getattr(self, "_" + attr))
        # send the message to the client
        msg.pack_msg_data()
        self._read_data = msg.msg_data

    def generateRandomData(self):
        sigma = 10

        if(self._angle == 0):
            self._noise = perlin.noise(400, 50, 50)

        mu = 100 + int(self._noise[self._angle]) + random.randint(-1, 1)

        self._data = "".join([chr(int(255 * np.exp(-np.power(x - mu, 2.) / (2 * np.power(sigma, 2.)))))
                              for x in range(self._number_of_samples)])

    #
    # Helpers for generating periodic data
    #

    def periodicFn(self, amplitude=0, offset=0, frequency=1.0, shift=0):
        return amplitude * math.sin(frequency * time.time() + shift) + offset

    def periodicFnInt(self, amplitude=0, offset=0, frequency=1.0, shift=0):
        return int(self.periodicFn(amplitude, offset, frequency, shift))
