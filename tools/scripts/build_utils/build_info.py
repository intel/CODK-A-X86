#!/usr/bin/env python

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

# Common code for build info generation

import os
import sys
import re
import binary_version_header

def get_section_size(f, section_name):
    decimal_size = 0
    re_res = re.search(r"(.*] "+section_name+".*)", f, re.MULTILINE)
    if re_res != None :
        size_group = re.split(" {2,}", re_res.group(1))[3]
        decimal_size = int(re.split("\s", size_group)[2], 16) if size_group != None else 0
    return decimal_size

def get_footprint_from_bin_and_statfile(bin_file, stat_file, total_flash, total_ram):
    """Compute flash and RAM memory footprint from a .bin and.stat file"""
    f = open(stat_file).read()

    # Get kctext + text + ctors + rodata + kcrodata segment size
    total_used_flash = os.path.getsize(bin_file)

    #getting used ram on target
    total_used_ram = (get_section_size(f, "noinit") + get_section_size(f, "bss")
        + get_section_size(f, "initlevel") + get_section_size(f, "datas") + get_section_size(f, ".data")
        + get_section_size(f, ".heap") + get_section_size(f, ".stack") + get_section_size(f, ".bss"))

    total_percent_ram = float(total_used_ram) / total_ram * 100
    total_percent_flash = float(total_used_flash) / total_flash * 100

    res = { "total_flash": total_used_flash,
            "percent_flash": total_percent_flash,
            "total_ram": total_used_ram,
            "percent_ram": total_percent_ram}
    return res

def get_binary_version_header_from_binfile(bin_file):
    """Search and extract a BinaryVersionHeader from a binary file"""
    arr = bytearray(open(bin_file).read())
    header_pos = arr.find(binary_version_header.MAGIC)
    if header_pos == -1:
        raise Exception("Cannot find the magic string %s in the passed binary." % binary_version_header.MAGIC)
    else:
        print "Found header magic at offset %x" % header_pos

    assert arr[header_pos:header_pos+4] == binary_version_header.MAGIC
    assert arr[header_pos+4] == binary_version_header.VERSION
    header = binary_version_header.BinaryVersionHeader.from_buffer(arr[header_pos:header_pos+48])
    return header
