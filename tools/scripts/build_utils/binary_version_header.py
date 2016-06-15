#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import ctypes
import hashlib
import binascii

MAGIC = "$B!N"
VERSION = 0x01

class BinaryVersionHeader(ctypes.Structure):
    """Binary version header with $B!N magic, see declaration in
       infra/version.h """
    _pack_ = 1
    _fields_ = [
        # Always equal to $B!N
        ("magic", ctypes.c_char * 4),

        # Header format version
        ("version", ctypes.c_ubyte),
        ("major", ctypes.c_ubyte),
        ("minor", ctypes.c_ubyte),
        ("patch", ctypes.c_ubyte),

        # Human-friendly version string, free format (not NULL terminated)
        # Advised format is: PPPPXXXXXX-YYWWTBBBB
        #  - PPPP  : product code, e.g ARD1
        #  - XXXXXX: binary info. Usually contains information such as the
        #    binary type (bootloader, application), build variant (unit tests,
        #    debug, release), release/branch name
        #  - YY    : year last 2 digits
        #  - WW    : work week number
        #  - T     : build type, e.g. [Engineering], [W]eekly, [L]atest,
        #    [R]elease, [P]roduction, [F]actory, [C]ustom
        #  - BBBB  : build number, left padded with zeros
        # Examples:
        #  - ARD1BOOT01-1503W0234
        #  - CLRKAPP123-1502R0013
        ("version_string", ctypes.c_ubyte * 20),

        # Micro-SHA1 (first 4 bytes of the SHA1) of the binary payload excluding
        # this header. It allows to uniquely identify the exact binary used.
        ("hash", ctypes.c_ubyte * 4),

        # Position of the payload relative to the address of this structure
        ("offset", ctypes.c_int32),
        ("reserved_1", ctypes.c_ubyte * 4),

        # Size of the payload, i.e. the full binary on which the hash was
        # computed (excluding this header). The beginning of the payload
        # is assumed to start right after the last byte of this structure.
        ("size", ctypes.c_uint32),
        ("reserved_2", ctypes.c_ubyte * 4)
    ]

    def get_printable_hash(self):
        return binascii.hexlify(self.hash)

    def get_printable_version_string(self):
        return ctypes.string_at(self.version_string, 20).partition(b'\0')[0]

