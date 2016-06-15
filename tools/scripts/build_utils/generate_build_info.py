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

# Generate a JSON file with useful info on the build like memory footprints size
# The JSON section for the given build directory is dumped to stdout
# The data is also merged in the given merge_out_file allowing to combine the
# build info for several targets in a single file.

import os
import sys
import json
import fcntl
import argparse

import custom_build_info

def main():
    parser = argparse.ArgumentParser(description="Generate a JSON file with useful"
        " info on the build like memory footprints size. The resulting JSON is "
        "merged with existing JSON data in merge_out_file and is also printed to"
        "screen.")
    parser.add_argument('build_dir', help="the directory where to look for build artifacts")
    parser.add_argument('merge_out_file', help="a file in which to merge the resulting JSON data")
    args = parser.parse_args()

    custom_info = custom_build_info.get_custom_info(args.build_dir)
    print json.dumps(custom_info, indent=4)

    build_setup = json.loads(open(args.build_dir + "/build_setup.json", "r").read())
    board = build_setup['BOARD']
    buildvariant = build_setup['BUILDVARIANT']
    project = build_setup['PROJECT']
    build_tag = build_setup['BUILD_TAG']
    download_dir_url = build_setup['DOWNLOAD_DIR_URL']

    # Merge the generated JSON into merge_out_file if it exists
    fd = os.open(args.merge_out_file, os.O_RDWR | os.O_CREAT)
    of = os.fdopen(fd, "r+")
    # Ensure that no other process is writing to this file at the same time
    # i.e.(blocks until other process is done)
    fcntl.lockf(of, fcntl.LOCK_EX)
    data = of.read()
    previousJson = {}
    if len(data)!=0:
        previousJson = json.loads(data)

    previousJson['project'] = project
    previousJson['build_tag'] = build_tag
    previousJson['download_dir_url'] = download_dir_url
    previousJson['manifest_file'] = 'manifest-'+build_tag+'.xml'

    # Append data specific to this variant build
    if 'variants' not in previousJson:
        previousJson['variants']={}
    variant = project+'-'+board+'-'+buildvariant
    if variant not in previousJson['variants']:
        previousJson['variants'][variant]={}
    previousJson['variants'][variant]['board'] = board
    previousJson['variants'][variant]['buildvariant'] = buildvariant
    previousJson['variants'][variant]['package_file'] = variant+'-'+build_tag+'.zip'
    previousJson['variants'][variant]['targets'] = custom_info

    # Append stuff for Phone Flash Tool
    if 'hardwares' not in previousJson:
        previousJson['hardwares']={}
    if board not in previousJson['hardwares']:
        previousJson['hardwares'][board] = {'variants': {}, 'hidden_to_end_user': False}
    previousJson['hardwares'][board]['variants'][buildvariant] = {'hardware_family': project, 'flashfiles': {'flash': [project+'-'+board+'-'+buildvariant+'-'+build_tag+'.zip:flash.json']}}

    of.seek(0, 0)
    of.write(json.dumps(previousJson, indent=4))

if __name__ == "__main__":
    sys.exit(main())
