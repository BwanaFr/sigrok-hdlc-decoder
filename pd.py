##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
## Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from collections import namedtuple

Data = namedtuple('Data', ['ss', 'es', 'val'])

'''
OUTPUT_PYTHON format:

Packet:
[<ptype>, <data1>, <data2>]

<ptype>:
 - 'DATA': <data1> contains the MOSI data, <data2> contains the MISO data.
   The data is _usually_ 8 bits (but can also be fewer or more bits).
   Both data items are Python numbers (not strings), or None if the respective
   channel was not supplied.
 - 'BITS': <data1>/<data2> contain a list of bit values in this MOSI/MISO data
   item, and for each of those also their respective start-/endsample numbers.
 - 'CS-CHANGE': <data1> is the old CS# pin value, <data2> is the new value.
   Both data items are Python numbers (0/1), not strings. At the beginning of
   the decoding a packet is generated with <data1> = None and <data2> being the
   initial state of the CS# pin or None if the chip select pin is not supplied.
 - 'TRANSFER': <data1>/<data2> contain a list of Data() namedtuples for each
   byte transferred during this block of CS# asserted time. Each Data() has
   fields ss, es, and val.

Examples:
 ['CS-CHANGE', None, 1]
 ['CS-CHANGE', 1, 0]
 ['DATA', 0xff, 0x3a]
 ['BITS', [[1, 80, 82], [1, 83, 84], [1, 85, 86], [1, 87, 88],
           [1, 89, 90], [1, 91, 92], [1, 93, 94], [1, 95, 96]],
          [[0, 80, 82], [1, 83, 84], [0, 85, 86], [1, 87, 88],
           [1, 89, 90], [1, 91, 92], [0, 93, 94], [0, 95, 96]]]
 ['DATA', 0x65, 0x00]
 ['DATA', 0xa8, None]
 ['DATA', None, 0x55]
 ['CS-CHANGE', 0, 1]
 ['TRANSFER', [Data(ss=80, es=96, val=0xff), ...],
              [Data(ss=80, es=96, val=0x3a), ...]]
'''

class ChannelError(Exception):
    pass

ann_bit, ann_data, ann_data_type, ann_warning, ann_transfer = range(5)
class Decoder(srd.Decoder):
    api_version = 3
    id = 'hdlc'
    name = 'HDLC'
    longname = 'High-Level Data Link Control'
    desc = 'High-Level Data Link Control.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['hdlc']
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'clk', 'name': 'CLK', 'desc': 'Clock'},
        {'id': 'data', 'name': 'DATA', 'desc': 'Data in'},
    )
    optional_channels = (
        {'id': 'en', 'name': 'ENABLE', 'desc': 'RX enabled'},
    )
    options = (
        {'id': 'en_polarity', 'desc': 'ENABLE polarity', 'default': 'active-high',
            'values': ('active-low', 'active-high')},
        {'id': 'cpol', 'desc': 'Clock polarity', 'default': 1,
            'values': (0, 1)},
    )
    annotations = (
        ('rx-bit', 'RX bit'),       #0: ann_bit 
        ('data', 'data'),           #1: ann_data
        ('data-type', 'data-type'), #2: ann_data_type
        ('warning', 'Warning'),     #3: ann_warning
        ('transfer', 'transfer'),   #4: ann_transfer
    )
    annotation_rows = (
        ('rx-bits', 'RX bits', (ann_bit,)),
        ('data-vals', 'data', (ann_data,ann_data_type,)),
        ('transfers', 'transfers', (ann_transfer,)),
        ('other', 'Other', (ann_warning,)),
    )
    binary = (
        ('data', 'DATA'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.bitcount = 0           # Actual shifted bit number
        self.rxdata = 0             # Received data byte
        self.rxbits = []            # Received bits (tuple, [start, end, value])
        self.rxbytes = []           # Received bytes (tuple, [start, end, value])
        self.prev_bit = None        # Previous bit state
        self.ss_prev_clock = -1     # Previous clock samplenum
        self.ss_prev_false = -1     # samplenum at previous false
        self.flag_found = False     # Start flag found
        self.ss_flag = -1           # samplenum at start flag
        self.have_en = None         # Use enable signal
        self.one_count = 0          # Number of consecutive 1
        self.prev_one_count = 0     # Previous number of consecutive 1

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.out_bitrate = self.register(srd.OUTPUT_META,
                meta=(int, 'Bitrate', 'Bitrate during transfers'))

    #Check if ENABLE is asserted
    def en_asserted(self, en):
        active_low = (self.options['en_polarity'] == 'active-low')
        return (en == 0) if active_low else (en == 1)

    #Computes frame CRC
    def crc16(data, length):
        if data is None and length > len(data):
            return 0
        crc = 0xFFFF
        for i in range(0, length):
            crc ^= data[i] << 0
            for j in range(0,8):
                if (crc & 0x0001) > 0:
                    crc =(crc >> 1) ^ 0x8408
                else:
                    crc = crc >> 1
        crc = (crc & 0xFFFF) ^ 0xFFFF
        return crc

    #Shifts bit
    def shift_bit(self, data):
        if self.flag_found:
            self.rxdata |= data << self.bitcount
            self.rxbits.append([data, self.samplenum])
            self.bitcount += 1

    #handles bit
    def handle_bit(self, clk, data, en):
        # Displays bit
        if(self.ss_prev_clock != -1):
            self.put(self.ss_prev_clock, self.samplenum, self.out_ann, [ann_bit, ['%d' % self.prev_bit]])
        self.ss_prev_clock = self.samplenum
        self.prev_bit = data

        #Flag
        if self.prev_one_count == 6:
            self.put(self.ss_flag, self.samplenum, self.out_ann, [ann_data_type, ['FLAG']])
        elif self.prev_one_count > 6:
            self.put(self.ss_flag, self.samplenum, self.out_ann, [ann_data_type, ['ABORT']])

        #Display previsous data
        if self.bitcount == 8:
            self.put(self.rxbits[0][1], self.samplenum, self.out_ann,
                                [ann_data, ['%02X' % self.rxdata]])
            self.rxdata = 0
            self.bitcount = 0
            self.rxbits = []

        #Count number of 1
        if data == 1:
            self.shift_bit(data)
            self.one_count += 1
        else:
            if self.one_count > 5:
                self.ss_flag = self.ss_prev_false
                self.flag_found = False

            if self.one_count == 6:
                self.flag_found = True
                self.rxdata = 0
                self.bitcount = 0
                self.rxbits = []
            elif self.one_count < 5:
                self.shift_bit(data)
            self.prev_one_count = self.one_count
            self.one_count = 0
            self.ss_prev_false = self.samplenum

    #Find clock edges
    def find_clk_edge(self, clk, data, en, first):
        # We only care about samples if CS# is asserted.
        if self.have_en and not self.en_asserted(en):
            self.rxdata = 0
            self.bitcount = 0
            self.rxbits = []
            self.rxbytes = []
            self.one_count = 0
            self.ss_prev_clock = -1
            return
        # Ignore sample if the clock pin hasn't changed.
        if first or not self.matched[0]:
            return

        #Check clock
        if self.options['cpol'] != clk:
            return

        self.handle_bit(clk, data, en)

    def decode(self):
        # The CLK input is mandatory. Other signals are (individually)
        # optional. Yet either MISO or MOSI (or both) must be provided.
        # Tell stacked decoders when we don't have a CS# signal.
        if not self.has_channel(0) and not self.has_channel(1):
            raise ChannelError('Both clock and data pins required.')
        self.have_en = self.has_channel(2)
       
        if not self.have_en:
            self.put(0, 0, self.out_python, ['CS-CHANGE', None, None])

        # We want all CLK changes. We want all CS changes if CS is used.
        # Map 'have_cs' from boolean to an integer index. This simplifies
        # evaluation in other locations.
        wait_cond = [{0: 'e'}]
        if self.have_en:
            self.have_en = len(wait_cond)
            wait_cond.append({3: 'e'})

        # "Pixel compatibility" with the v2 implementation. Grab and
        # process the very first sample before checking for edges. The
        # previous implementation did this by seeding old values with
        # None, which led to an immediate "change" in comparison.
        (clk, data, en) = self.wait({})
        self.find_clk_edge(clk, data, en, True)

        while True:
            (clk, data, en) = self.wait(wait_cond)
            self.find_clk_edge(clk, data, en, False)
