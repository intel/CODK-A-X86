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
import sys
import struct
import re
import shutil

FW_MODULE_NAME= sys.argv[1]
fileSize = int(sys.argv[2], 10)
BREAK_AFTER_RESET = int(sys.argv[3], 10)

# Defines
OUTFILE     = "FSRom.bin"
OUTFILE1    = "FSRomNoPatch.bin"
RESETVEC16F = "ResetVec.com"
FLAT32F     = FW_MODULE_NAME + ".com"

# 8KiB of ROM minus 1KiB of factory data
FWFileSize = 0x1C00

ResetVecStartOffset = 0x0
# See the linker file gcc4.4-ld-script.ld for the corresponding offset
Flat32StartOffset = 0x0

currdir         = os.getcwd()
newRomFile      = open(currdir + '/' + OUTFILE, 'wb+')
newRomFile1      = open(currdir + '/' + OUTFILE1, 'wb+')
ResetVecFile    = open(currdir + '/' + RESETVEC16F, 'rb').read()
pFlat32File     = open(currdir + '/' + FLAT32F, 'rb').read()

ResetVecLen = len(ResetVecFile)
Flat32Len = len(pFlat32File)

#4K  offset = 0x1000
#8K  ofsset = 0x2000
Flat32Offset = FWFileSize - 0x1C00

padRomLength = Flat32Offset
print '**************'
print padRomLength
print '**************'

currCount = 0

for currCount in range(padRomLength):
    newRomFile.write('\x00')
    currCount = currCount+1

#currCount = smrtosLen
print '**CurrCount*****'
print currCount
print FWFileSize - currCount
print 0xFFFFFFFF - (FWFileSize - currCount)
print '**************'

# Pad 1k for OTP and data storage area
for currCount in range(Flat32StartOffset):
    newRomFile.write('\xFF')
    currCount = currCount+1

# add Flat 32
newRomFile.write(pFlat32File)
currCount = currCount + Flat32Offset + Flat32Len

# Subtract RSV from end
currCount = currCount + ResetVecLen
padRomLength = FWFileSize - currCount

#print '----padRomLength-------'
#print "ResetVecLen = " + str(ResetVecLen)
#print "Flat32Offset = " + str(Flat32Offset)
#print "Flat32Len = " + str(Flat32Len)
#print "FWFileSize = " + str(FWFileSize)
#print "padRomLength = " + str(padRomLength)
#print "currCount = " + str(currCount)
#print '------------------'
for romLenCount in range(padRomLength):
    newRomFile.write('\x00')
    currCount = currCount+1

# Add Reset Vec
newRomFile.write(ResetVecFile)


newRomFile.seek(0, os.SEEK_SET)
currentBinLength = len(newRomFile.read())
newRomFile.seek(0, os.SEEK_SET)
newRomFile1.write(newRomFile.read())

# patch Reset Vec
newRomFile.seek(0, os.SEEK_SET)
currentBinLength = len(newRomFile.read())
newRomFile.seek(0, os.SEEK_SET)
checkString="FLAT32S_ENTRY"
print checkString + '............. = ' + str(currentBinLength)

for currCount in range(currentBinLength):
    currByte = newRomFile.read(1)
    currCount = currCount+1

    for currChar in checkString:
        if currByte == currChar:
            currByte = newRomFile.read(1)
            currCount = currCount+1
            print 'currChar = ' + currChar
        else:
            break;

        if 'Y' == currChar:
            #"Actually want the byte after this"
            print "#############"
            mySpiLocation = newRomFile.tell() -1
            print hex(mySpiLocation)
            print hex(mySpiLocation)
            print "#############"
            #E9<LSB><MSB>

#############
#0x100c
#############
            relSpiLocation = FWFileSize - mySpiLocation
            TopAddress = 0x10000
            addRelSpiLocation = TopAddress - relSpiLocation

            print hex(addRelSpiLocation)
#############
#0xeFF4
#############

            if 1 == BREAK_AFTER_RESET:
                eipLoc=0xFFF2 + 6

                jumpDist = eipLoc - addRelSpiLocation
                print hex(jumpDist)

                addRelSpiLocation = (0xFFFF ^ jumpDist) + 1
                print hex(addRelSpiLocation)

                newRomFile.seek(-14, os.SEEK_END)
                newRomFile.write(struct.pack('B', 0x90))
                newRomFile.write(struct.pack('B', 0xeb))
                newRomFile.write(struct.pack('B', 0xfd))
                newRomFile.write(struct.pack('B', 0xe9))
                newRomFile.write(struct.pack('B', addRelSpiLocation & 0xFF))
                newRomFile.write(struct.pack('B', (addRelSpiLocation >> 8)  & 0xFF))
            else:
                eipLoc=0xFFF2 + 3

                jumpDist = eipLoc - addRelSpiLocation
                print hex(jumpDist)

                addRelSpiLocation = (0xFFFF ^ jumpDist) + 1
                print hex(addRelSpiLocation)

                newRomFile.seek(-13, os.SEEK_END)
                newRomFile.write(struct.pack('B', addRelSpiLocation & 0xFF))
                newRomFile.write(struct.pack('B', (addRelSpiLocation >> 8)  & 0xFF))

            break;



newRomFile.close()
print "Done!"
