#!/usr/bin/env python2

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

import os
import sys
import argparse
import subprocess


# Return a dict containing symbol_name: path/to/file/where/it/originates
# for all symbols from the .elf file. Optionnaly strips the path according
# to the passed sub-path
def load_symbols_and_paths(elf_file, path_to_strip = None):
    symbols_paths = {}
    nm_out = subprocess.check_output(["nm", elf_file, "-S", "-l", "--size-sort", "--radix=d"])
    for line in nm_out.split('\n'):
        fields = line.replace('\t', ' ').split(' ')
        # Get rid of trailing empty field
        if len(fields) == 1 and fields[0] == '':
            continue
        assert len(fields)>=4
        if len(fields)<5:
            path = ":/" + fields[3]
        else:
            path = fields[4].split(':')[0]
        if path_to_strip != None:
            if path_to_strip in path:
                path = path.replace(path_to_strip, "") + '/' + fields[3]
            else:
                path = ":/" + fields[3]
        symbols_paths[fields[3]] = path
    return symbols_paths


# Return a JavaScript snippet necessary for drawing a treemap diagram.
# To display the diagram, the HTML code needs to add a div with an ID
# equal to the div_name argument
def create_java_script_for_table(table, div_name):
    out = """<script type="text/javascript">
      google.load("visualization", "1", {packages:["treemap"]});
      google.setOnLoadCallback(drawChart);
      function drawChart() {
        var data = google.visualization.arrayToDataTable(["""

    for t in table:
        out += "[" + t[0] + ", " + t[1] + ", " + t[2] + ", " + t[3] + "],\n"

    out += """]);
        tree = new google.visualization.TreeMap(document.getElementById('"""+ div_name +"""'));

        tree.draw(data, {
minHighlightColor: '#8c6bb1',
        midHighlightColor: '#9ebcda',
        maxHighlightColor: '#edf8fb',
        minColor: '#009688',
        midColor: '#f7f7f7',
        maxColor: '#ee8100',
        headerHeight: 20,
        fontColor: 'black',
        showScale: true,
        generateTooltip: showSize,
        title: 'Footprint Details (Left click to digg in the elf file, right click to go up)',
      });

    function showSize(row, size, value) {
        return '<div style="background:#fd9; padding:10px; border-style:solid">' +
           '<span style="font-family:Courier"><b> ' + data.getValue(row, 0) + ': ' + size + ' Bytes</b> </div>';
    }

      }
    </script>\n"""
    return out



def main():
    parser = argparse.ArgumentParser(description="""Output an HTML report with
    the analyze of where the size on flash and on RAM of the ELF file is spent""")
    parser.add_argument('elf_file', help="the elf file to analyze")
    parser.add_argument('bin_file', help="the actual .bin file generated from the elf")
    parser.add_argument('path_to_strip', help="part of the paths to strip for symbols and files")
    parser.add_argument('path_to_strip_for_bin_file', help="part of the paths to strip for .bin and .elf file in  summary")
    args = parser.parse_args()

    bin_file_stripped = args.bin_file
    elf_file_stripped = args.elf_file
    if args.path_to_strip_for_bin_file != None:
        bin_file_stripped = args.bin_file.replace(args.path_to_strip_for_bin_file, "")
        elf_file_stripped = args.elf_file.replace(args.path_to_strip_for_bin_file, "")

     # First deal with size on flash. These are the symbols flagged as LOAD in objdump output
    size_out = subprocess.check_output(["objdump", "-hw", args.elf_file])
    loaded_section_total = 0
    loaded_section_names = []
    loaded_section_names_sizes = {}
    ram_section_total = 0
    ram_section_names = []
    ram_section_names_sizes = {}
    for line in size_out.split('\n'):
        if "LOAD" in line:
            loaded_section_total = loaded_section_total + int(line.split()[2], 16)
            loaded_section_names.append(line.split()[1])
            loaded_section_names_sizes[line.split()[1]] = int(line.split()[2], 16)
        if "ALLOC" in line and "READONLY" not in line and "CODE" not in line:
            ram_section_total = ram_section_total + int(line.split()[2], 16)
            ram_section_names.append(line.split()[1])
            ram_section_names_sizes[line.split()[1]] = int(line.split()[2], 16)

    # Actual .bin size, which doesn't not always match section sizes
    bin_size = os.stat(args.bin_file).st_size

    # Get the path associated to each symbol
    symbols_paths = load_symbols_and_paths(args.elf_file, args.path_to_strip)

    # A set of helper function for building a simple tree with a path-like
    # hierachy.
    def _insert_one_elem(tree, path, size):
        splitted_path = path.split('/')
        cur = None
        for p in splitted_path:
            if cur == None:
                cur = p
            else:
                cur = cur + '/' + p
            if cur in tree:
                tree[cur] += size
            else:
                tree[cur] = size

    def _parent_for_node(e):
        parent = "root" if len(e.split('/')) == 1 else e.rsplit('/', 1)[0]
        if e == "root":
            parent = None
        return parent

    def _childs_for_node(tree, node):
        res = []
        for e in tree:
            if _parent_for_node(e) == node:
                res += [e]
        return res

    def _siblings_for_node(tree, node):
        return _childs_for_node(tree, _parent_for_node(node))

    def _max_sibling_size(tree, node):
        siblings = _siblings_for_node(tree, node)
        return max([tree[e] for e in siblings])


    # Extract the list of symbols a second time but this time using the objdump tool
    # which provides more info as nm
    symbols_out = subprocess.check_output(["objdump", "-tw", args.elf_file])
    flash_symbols_total = 0
    data_nodes = {}
    data_nodes['root'] = 0

    ram_symbols_total = 0
    ram_nodes = {}
    ram_nodes['root'] = 0
    for l in symbols_out.split('\n'):
        line = l[0:9] + "......." + l[16:]
        fields = line.replace('\t', ' ').split(' ')
        # Get rid of trailing empty field
        if len(fields) != 5:
            continue
        size = int(fields[3], 16)
        if fields[2] in loaded_section_names and size != 0:
            flash_symbols_total += size
            _insert_one_elem(data_nodes, symbols_paths[fields[4]], size)
        if fields[2] in ram_section_names and size != 0:
            ram_symbols_total += size
            _insert_one_elem(ram_nodes, symbols_paths[fields[4]], size)


    out = """<!DOCTYPE html>
    <html>
    <head>
    <title>Memory Analysis for binary: %s</title>
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
    <script type="text/javascript" src="https://www.google.com/jsapi"></script>""" % bin_file_stripped

    # Generate a table for elements on flash, used for the flash usage treemap
    table = []
    table += [[ "'Node'", "'Parent'", "'Size'", "'Color'" ]]
    table += [[ "'data without symbols (strings, etc..)'", "'root'", '%d' % (loaded_section_total-flash_symbols_total), '-100' ]]
    table += [[ "'.bin overhead (section alignment etc..)'", "'root'", '%d' % (bin_size-loaded_section_total), '-80' ]]
    for e in data_nodes:
        parent = _parent_for_node(e)
        table += [[ "'" + e + "'", ("'" + parent + "'" if parent != None else "null"), "%d" % data_nodes[e], "%d" % (data_nodes[e]%100-50) ]]
    out += create_java_script_for_table(table, "chart_div")

    # Generate a table for elements on RAM, used for the RAM usage treemap
    table = []
    table += [[ "'Node'", "'Parent'", "'Size'", "'Color'" ]]
    for e in ram_nodes:
        parent = _parent_for_node(e)
        table += [[ "'" + e + "'", ("'" + parent + "'" if parent != None else "null"), "%d" % ram_nodes[e], "%d" % (ram_nodes[e]%100-50) ]]
    out += create_java_script_for_table(table, "chart_div_ram")


    # Create a simplified tree keeping only the most important contributors
    # This is used for the pie diagram summary
    min_parent_size = bin_size/25
    min_sibling_size = bin_size/35
    tmp = {}
    for e in data_nodes:
        if _parent_for_node(e) == None:
            continue
        if data_nodes[_parent_for_node(e)] < min_parent_size:
            continue
        if _max_sibling_size(data_nodes, e) < min_sibling_size:
            continue
        tmp[e] = data_nodes[e]

    # Keep only final nodes
    tmp2 = {}
    for e in tmp:
        if len(_childs_for_node(tmp, e)) == 0:
            tmp2[e] = tmp[e]

    # Group nodes too small in an "other" section
    filtered_data_nodes = {}
    for e in tmp2:
        if tmp[e] < min_sibling_size:
            k = _parent_for_node(e) + "/(other)"
            if k in filtered_data_nodes:
                filtered_data_nodes[k] += tmp[e]
            else:
                filtered_data_nodes[k] = tmp[e]
        else:
            filtered_data_nodes[e] = tmp[e]

    def _parent_level_3_at_most(node):
        e = _parent_for_node(node)
        while e.count('/')>2:
            e = _parent_for_node(e)
        return e

    # Try to use the same color for stuff comming from the same directory at level 3
    colors = ["#{0:6x}".format(abs(hash(_parent_level_3_at_most(e))))[:7] for e in sorted(filtered_data_nodes.keys())]
    color_str = "["
    for c in colors:
        color_str += "{color: '" + c + "'},"
    color_str += "]"

    table = []
    for e in sorted(filtered_data_nodes.keys()):
        table += [[e, filtered_data_nodes[e]]]

    out += """<script type="text/javascript">
      google.load("visualization", "1", {packages:["corechart"]});
      google.setOnLoadCallback(drawChartPie);
      function drawChartPie() {
        var data = google.visualization.arrayToDataTable(["""
    out += "['Node', 'Size (Bytes)'],\n"
    for e in table:
        out += "['" + e[0] + "', %d],\n" % e[1]
    out += "])\n"
    out += """var options = {
          title: 'Flash Footprint Summary',
          pieHole: 0.5,
          slices: %s
        };

        var chart = new google.visualization.PieChart(document.getElementById('piechart_3d'));
        chart.draw(data, options);
      }
    </script>\n""" % color_str

    out += """</head>
    <body>
    <h1>Memory analysis for binary: %s</h1>
    <h2>Size on flash</h2>
    <p>Actual bin file size: %s Bytes composed of:</p>
    <ul>
        <li>Sections:
            <ul>""" % (bin_file_stripped, bin_size)

    for i in loaded_section_names_sizes:
        out += "                <li>%s: %d Bytes</li>" % (i, loaded_section_names_sizes[i])

    out += "                <li><b>total</b>: %d Bytes</li>" % loaded_section_total
    out += """            </ul>
        <li>.bin overhead (section alignment etc..): %d Bytes</li>
    </ul>
    <p>Data without symbols (strings, etc..) size: %d Bytes</p>
    <div id="piechart_3d" style="width: 900px; height: 500px;"></div>
    <div id="chart_div" style="width: 1024px; height: 768px;"></div>""" % (bin_size-loaded_section_total, loaded_section_total-flash_symbols_total)

    out += """<h2>Size on RAM</h2>
    <p>Total RAM usage: %d Bytes, from the following sections:</p>
    <ul>""" % ram_section_total

    for i in ram_section_names_sizes:
        out += "<li>%s: %d Bytes</li>" % (i, ram_section_names_sizes[i])
    out += """</ul>
    <div id="chart_div_ram" style="width: 1024px; height: 768px;"></div>
  </body>
</html>"""

    print out

if __name__ == "__main__":
    sys.exit(main())
