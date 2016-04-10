#
# Copyright 2013 Free Software Foundation, Inc.
#
# This file is part of GNU Radio.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import zmq
import threading
import numpy
import pmt

class probe_manager():
    def __init__(self):
        self.zmq_context = zmq.Context()
        self.poller = zmq.Poller()
        self.interfaces = []

    def add_socket(self, address, data_type, callback_func):
        socket = self.zmq_context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, "")
        socket.connect(address)
        # use a tuple to store interface elements
        self.interfaces.append((socket, data_type, callback_func))
        self.poller.register(socket, zmq.POLLIN)

    def watcher(self):
        poll = dict(self.poller.poll(100))
        for i in self.interfaces:
            # i = (socket, data_type, callback_func)
            if poll.get(i[0]) == zmq.POLLIN:
                # receive data
                msg_packed = i[0].recv()
                # use numpy to unpack header and data
                header_magic = numpy.fromstring(msg_packed[:2], numpy.dtype('uint16'))
                #print "header_magic",header_magic
                if header_magic != 0x5FF0:
                    raise ValueError('Error: gr-zmq header magic does not match!')
                header_version = numpy.fromstring(msg_packed[2:3], numpy.dtype('uint8'))
                #print "header_version",header_version
                if header_version != 1:
                    raise ValueError('Error: gr-zmq header version too high!')
                offset_out = numpy.fromstring(msg_packed[3:11], numpy.dtype('uint64'))
                #print "offset_out",offset_out
                rcv_ntags = numpy.fromstring(msg_packed[11:19], numpy.dtype('uint64'))
                #print "rcv_ntags",rcv_ntags

                if rcv_ntags == 3:
                    # extract all the tags
                    offset = numpy.fromstring(msg_packed[19:27], numpy.dtype('uint64'))
                    #print "offset", offset
                    key = str(pmt.deserialize_str(msg_packed[27:37]))
                    #print "key",key
                    value = pmt.to_python(pmt.deserialize_str(msg_packed[37:60]))
                    value = value[0] + value[1]
                    #print "value", value
                    srcid = str(pmt.deserialize_str(msg_packed[60:82]))
                    #print "srcid",srcid

                    key_2 = str(pmt.deserialize_str(msg_packed[90:100]))
                    #print "key_2",key_2
                    value_2 = pmt.to_float(pmt.deserialize_str(msg_packed[100:109]))
                    #print "value_2", value_2
                    srcid_2 = str(pmt.deserialize_str(msg_packed[109:131]))
                    #print "srcid_2",srcid_2

                    key_3 = str(pmt.deserialize_str(msg_packed[139:149]))
                    #print "key_3",key_3
                    value_3 = pmt.to_float(pmt.deserialize_str(msg_packed[149:158]))
                    #print "value_3", value_3
                    srcid_3 = str(pmt.deserialize_str(msg_packed[158:180]))
                    #print "srcid_3",srcid_3

                    # Reverse engineering (brute force)
                    #for n in range(39,39+1*64):
                    #for m in range(145,1):
                    #    for n in range(160,180):
                    #        try:
                    #            key = pmt.deserialize_str(msg_packed[m:n])
                    #            print "value_3",key
                    #            print m,n
                    #        except:
                    #            pass

                    samples = numpy.fromstring(msg_packed[180:], numpy.dtype(i[1]))
                    tags = {key:value,key_2:value_2,key_3:value_3}
                else:
                    samples = numpy.fromstring(msg_packed[19:], numpy.dtype(i[1]))
                    tags = None

                # invoke callback function
                i[2](samples,tags)
