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
import sys
import argparse
import hashlib
import binary_version_header

def main(argv):
    parser = argparse.ArgumentParser(description="""Overwrite the binary
        version header in the passed binary file.""")
    parser.add_argument('--major', dest='major', type=int,
        help='major version number', required=True)
    parser.add_argument('--minor', dest='minor', type=int,
        help='minor version number', required=True)
    parser.add_argument('--patch', dest='patch', type=int,
        help='patch version number', required=True)
    parser.add_argument('--version_string', dest='version_string',
        help='human friendly version string, free format', required=True)
    parser.add_argument('--header_position_hint', dest='header_position_hint',
        type=str, help='one of start, end, unknown. Default to unknown.',
        default="unknown")
    parser.add_argument('input_file',
        help='the binary file to modify in place')
    args = parser.parse_args()

    arr = bytearray(open(args.input_file, "rb").read())

    if args.header_position_hint == 'start':
        header_pos = 0
    elif args.header_position_hint == 'end':
        header_pos = len(arr)-48
    elif args.header_position_hint == 'unknown':
        header_pos = arr.find(binary_version_header.MAGIC)
        if header_pos == -1:
            raise Exception("Cannot find the magic string %s in the passed binary." % binary_version_header.MAGIC)
        else:
            print "Found header magic at offset %x" % header_pos
    else:
        raise Exception("Invalid value for header_position_hint argument.")

    assert arr[header_pos:header_pos+4] == binary_version_header.MAGIC
    assert arr[header_pos+4] == binary_version_header.VERSION

    # Compute the hash. The header can be anywhere within the binary so we feed
    # the generator in 2 passes
    m = hashlib.sha1()
    m.update(arr[0:header_pos])
    m.update(arr[header_pos+48:])
    digest = bytearray(m.digest())

    # Create and initialize our header struct
    bh = binary_version_header.BinaryVersionHeader(binary_version_header.MAGIC, 0x01)
    assert len(bytearray(bh)) == 48
    bh.major = args.major
    bh.minor = args.minor
    bh.patch = args.patch
    for i in range(0, 4):
        bh.hash[i] = digest[i]
    if len(args.version_string) < 20:
        args.version_string += '\0' * (20-len(args.version_string))
    vs = bytearray(args.version_string)
    for i in range(0, 20):
        bh.version_string[i] = vs[i]
    bh.offset = -header_pos
    bh.size  = len(arr)

    # Over-write the header content
    arr[header_pos:header_pos+48] = bytearray(bh)

    # And save it
    out_file = open(args.input_file, "wb")
    out_file.write(arr)

if __name__ == "__main__":
    main(sys.argv)
