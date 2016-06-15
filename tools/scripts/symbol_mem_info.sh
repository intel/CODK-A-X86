#!/bin/sh
# this script is used by parse_symbol_mem_info.sh
# can not be used alone.

if [ $# -eq 1 ]
then
	obj_path=$1
else
	exit 1
fi

out_rodata=$(cat out/mem_rodata_symbols.csv | grep $obj_path | awk '{s+=$1} END {print s}')
out_bss=$(cat out/mem_bss_symbols.csv       | grep $obj_path | awk '{s+=$1} END {print s}')
out_text=$(cat out/mem_text_symbols.csv     | grep $obj_path | awk '{s+=$1} END {print s}')

if [ -f $obj_path ]
then
	base=". . . . . . File: "$(basename $obj_path)
else
	base=[$obj_path]
fi

if [ -z $out_bss ]
then
	out_bss=0
fi

if [ -z $out_text ]
then
	out_text=0
fi

if [ -z $out_rodata ]
then
	out_rodata=0
fi

typeset -i sum_t
typeset -i sum_without_bss

sum_t=$out_rodata+$out_text+$out_bss
sum_without_bss=$out_rodata+$out_text

if [ $sum_t -ne 0 ]
then
	echo $base, $out_rodata, $out_text, $out_bss, $sum_without_bss
fi
