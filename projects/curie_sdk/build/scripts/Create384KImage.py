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

import glob
import os
import subprocess
from subprocess import (PIPE, Popen)
import getopt
import sys
import struct
import re
import shutil

FWFileSize384K = 0x60000

def main(argv):
    IA_IMAGE_NAME = ''
    SS_IMAGE_NAME = ''
    OP_IMAGE_NAME = ''
    try:
        opts, args = getopt.getopt(argv,"I:S:O:")
    except getopt.GetoptError:
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-O"):
            OP_IMAGE_NAME = str(arg)
        if opt in ("-I"):
            IA_IMAGE_NAME = str(arg)
        if opt in ("-S"):
            SS_IMAGE_NAME = str(arg)

    if '' == OP_IMAGE_NAME:
        print 'use -O to set the output image name'
        exit(2)
    else:
        FlashFSTFile = open(OP_IMAGE_NAME, 'wb+')

    for x in range(FWFileSize384K >> 2):
    #    FlashFSTFile.write(struct.pack('I', 0xEFBEADDE))
        FlashFSTFile.write(struct.pack('I', 0x00000000))

    if '' != IA_IMAGE_NAME:
        IARAMFile = open(IA_IMAGE_NAME, 'rb').read()
        FlashFSTFile.seek(0x30000, os.SEEK_SET)
        FlashFSTFile.write(IARAMFile)
    if '' != SS_IMAGE_NAME:
        SSRAMFile = open(SS_IMAGE_NAME, 'rb').read()
        FlashFSTFile.seek(0, os.SEEK_SET)
        FlashFSTFile.write(SSRAMFile)




if __name__ == "__main__":
   main(sys.argv[1:])

