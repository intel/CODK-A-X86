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

# Flash the latest build and analyze the console output to check if:
# - the flash succeeded
# - the boot succeeded without panic
# - the board respond to a tcmd on reference_app.
# possible results :
#    FLASH FAILED  - flash failed
#    PASSED        - tcmd passed
#    PANIC         - in case of panic during testing
#    HANGED        - board does not respond anymore

import os
import sys
import json
import subprocess
import argparse
import fcntl
import threading
import time
from signal import alarm, signal, SIGALRM, SIGKILL
from subprocess import Popen, PIPE, call, check_output
from tests_library import *

def main():
    parser = argparse.ArgumentParser(description="Flash the latest build and "
        "analyze the console output to check if:"
        "FLASH FAILED  - flash failed"
        "PASSED        - tcmd passed"
        "PANIC         - in case of panic during testing"
        "HANGED        - board does not respond anymore")
    parser.add_argument('root_dir', help="the top level directory (containing out/current after a build)")
    args = parser.parse_args()

    flash_result = flash(args.root_dir)
    # stop tests if the flash failed
    if(flash_result == 0):
        ttyUSB = usb_manager()
        s = ttyUSB_read (ttyUSB, timeout=15)

        tty = os.open(ttyUSB, os.O_RDWR)

        currentTime = time.time()
        # send "version get" tcmd
        os.write(tty,"version get\n")
        # save log for 2s
        while((time.time() - currentTime) < 2):
            strRead = os.read(tty, 200)
            print strRead,
            s += strRead
        os.close(tty)

        # check logs to return results
        if s.find('Panic') != -1:
            result = "PANIC"
        elif s.find('Micro-sha1') != -1:
            result = "PASSED"
        else:
            result = "HANGED"
    else:
        result = "FLASH_FAILED"

    print "reference app result: " + result


    if (result != "PASSED"):
        return -1

if __name__ == "__main__":
    sys.exit(main())

