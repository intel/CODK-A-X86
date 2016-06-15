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

# Custom methods for Quark_SE build_info generation

import os
import sys
import json
import build_info


QUARK_FLASH = 147456 # 144 KiB
QUARK_RAM   =  56320 #  55 KiB
ARC_FLASH      = 155648 # 152 KiB
ARC_RAM        =  24576 #  24 KiB
TOTAL_FLASH    = QUARK_FLASH + ARC_FLASH
TOTAL_RAM      = QUARK_RAM   + ARC_RAM

def get_custom_info(build_dir):
    """ Compute various info for each sub-targets (quark, arc and nordic).
        This code is the only project-specific bit used by most scripts.
    """
    build_setup = json.loads(open(build_dir + "/build_setup.json", "r").read())
    buildvariant = build_setup['BUILDVARIANT']

    quark_file = 'firmware/quark'
    quark_binheader = build_info.get_binary_version_header_from_binfile(os.path.join(build_dir, quark_file+'.bin'))

    arc_file = 'firmware/arc'
    arc_binheader = build_info.get_binary_version_header_from_binfile(os.path.join(build_dir, arc_file+'.bin'))

    bootloader_file = 'firmware/bootloader_quark'
    bootloader_binheader = build_info.get_binary_version_header_from_binfile(os.path.join(build_dir, bootloader_file+'.bin'))

    return {
        "quark": {
            "footprint": build_info.get_footprint_from_bin_and_statfile(os.path.join(build_dir, quark_file+'.bin'),
                os.path.join(build_dir, quark_file+'.stat'), QUARK_FLASH, QUARK_RAM),
            "hash": quark_binheader.get_printable_hash(),
            "version_string": quark_binheader.get_printable_version_string(),
            "bin_file": quark_file+'.bin',
            "elf_file": quark_file+'.elf'
        },
        "arc": {
            "footprint": build_info.get_footprint_from_bin_and_statfile(os.path.join(build_dir, arc_file+'.bin'),
                os.path.join(build_dir, arc_file+'.stat'), ARC_FLASH, ARC_RAM),
            "hash": arc_binheader.get_printable_hash(),
            "version_string": arc_binheader.get_printable_version_string(),
            "bin_file": arc_file+'.bin',
            "elf_file": arc_file+'.elf'
        },
        "bootloader": {
            "footprint": build_info.get_footprint_from_bin_and_statfile(os.path.join(build_dir, bootloader_file+'.bin'),
                os.path.join(build_dir, bootloader_file+'.stat'), QUARK_FLASH, QUARK_RAM),
            "hash": bootloader_binheader.get_printable_hash(),
            "version_string": bootloader_binheader.get_printable_version_string(),
            "bin_file": bootloader_file+'.bin',
            "elf_file": bootloader_file+'.elf'
        }
    }
