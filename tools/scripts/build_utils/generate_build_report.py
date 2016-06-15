#!/usr/bin/env/python2

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

# Prints on stdout an HTML report generated from the input JSON files.

import os
import sys
import json
from jinja2 import Template, Environment
import argparse
from sets import Set
import copy

def get_board_list(data):
    if data == None:
        return None
    ret = Set()
    try:
        for variant, variant_desc in data.items():
            ret.add(variant_desc['board'])
        return ret
    except:
        return None

def filter_by_board(data, board):
    if data == None:
        return None
    ret = copy.deepcopy(data)
    try:
        for variant, variant_desc in ret['variants'].items():
            if variant_desc['board'] != board:
                del ret['variants'][variant]
        return ret
    except:
        return None

def render_one(template, build_data, build_ref_data):
    try:
        print template.render(data=build_data, ref_data=build_ref_data)
    except:
        print "<p><b>Warning:</b> There was an error while generating build report, this may be caused by a change of format between the new build report (%s) and the one used as reference (%s). In some case, it is sufficient to use a consistent reference file by rebuilding to solve this issue</p>" % (args.input_json, args.input_ref_json)
        print template.render(data=build_data, ref_data=None)

def main():
    parser = argparse.ArgumentParser(description="Prints on stdout an HTML "
        "report generated from the input JSON files.")
    parser.add_argument('input_json', help="the path to the build info JSON "
        "file for a given build")
    parser.add_argument('--input_ref_json', help="optional path to the build "
        "info JSON file for a reference build to which we want to compare, e.g."
        "to track the evolution of the footprint sizes", default="")
    args = parser.parse_args()

    build_data = json.loads(open(args.input_json, 'r').read())
    build_ref_data = None
    try:
        build_ref_data = json.loads(open(args.input_ref_json, 'r').read())
    except:
        pass

    env = Environment(trim_blocks=True, lstrip_blocks=True)
    template = env.from_string("""
    <table class="pane bigtable">
    <tbody><tr>
    <th class="pane-header">Projects</th>
    {% for variant in data['variants']|dictsort %}
        <th class="pane-header">{{ variant[0] }}</th>
    {% endfor %}
    </tr>
    <tr>
    <th class="pane-header">Targets</th>
    {% for variant in data['variants']|dictsort %}
    <td class="pane">
    {% for target, target_desc in variant[1]['targets'].iteritems() %}
        <b>{{ target }}:</b>
        <ul>
            {%- set has_ref = ref_data and ref_data['variants'][variant[0]] %}
            {%- set flash_diff = target_desc.footprint.total_flash - ref_data['variants'][variant[0]]['targets'][target]['footprint'].total_flash if has_ref else 0 %}
            {%- set flash_diff_str = " [==]" if flash_diff==0 else " [%+d]"|format(flash_diff) %}
            {%- set ram_diff = target_desc.footprint.total_ram - ref_data['variants'][variant[0]]['targets'][target]['footprint'].total_ram if has_ref else 0 %}
            {%- set ram_diff_str = " [==]" if ram_diff==0 else " [%+d]"|format(ram_diff) %}
            <li>used flash: {{ target_desc.footprint.total_flash }} <small>{{ flash_diff_str if has_ref }}</small></li>
            <li>% used flash: {{ "%.2f"|format(target_desc.footprint.percent_flash) }} %</li>
            <li>used ram: {{ target_desc.footprint.total_ram }}<small>{{ ram_diff_str if has_ref }}</small></li>
            <li>% used ram: {{ "%.2f"|format(target_desc.footprint.percent_ram) }} %</li>
            <li>hash: {{ target_desc.hash }} {{ "<small>[==]</small>" if (has_ref and ref_data['variants'][variant[0]]['targets'][target].hash == target_desc.hash) }} </li>
            <li>version_string: {{ target_desc.version_string }} </li>
            <li><a href="{{data['download_dir_url']}}/build_report/{{variant[0]}}_{{target}}_footprint_report-{{data['build_tag']}}.html">memory analysis</a></li>
        </ul>
    {% endfor %}
    </td>
    {% endfor %}
    </tr>
    </tbody></table>
    """)

    try:
        if len(build_data['variants'])>5:
            for board in get_board_list(build_data['variants']):
                render_one(template, filter_by_board(build_data, board), filter_by_board(build_ref_data, board))
        else:
            render_one(template, build_data, build_ref_data)
    except:
        print "<p><b>Warning:</b> There was an error while generating build report, this may be caused by a change of format. In some case, it is sufficient to rebuild to solve this issue</p>"
        pass

if __name__ == "__main__":
    sys.exit(main())
