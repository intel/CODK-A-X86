#!/bin/sh
# this script extract memory usage on ELF file
# need to be run from a repo and the root directory
# input file: elf file
# output file: csv files located in ./out/ folder

if [ $# -eq 3 ];
then
	elf_file=$1
	out_file=$2
	level=$3
else
	echo "    Usage: $0 [elf_file] [output file] [level]"
	echo "      [level]: used for 'high level' file info, this is the max depth for folder"
	echo "               possible value is [0...MaxDepth], 0 mean no high level file"
	echo "    Need to be started on main folder"
	echo "    Ex: ./arduino101_firmware/tools/scripts/parse_symbol_mem_info.sh ./out/quark_se_reference_ctb_release/firmware/quark.elf mem_quark.csv 0"
	exit 1
fi

basepath=$(pwd)/arduino101_firmware/tools/scripts

if [ -f $basepath ]
then
	echo "Error, please start this script in main folder"
        echo "Ex: ./arduino101_firmware/tools/scripts/parse_symbol_mem_info.sh ./out/quark_se_reference_ctb_release/firmware/quark.elf mem_quark.csv 0"
	exit 1
fi

check_path=$(pwd)/out
if ! [ -e $check_path ]
then
	echo "Error, please start a build first, out folder not found"
	exit 1
fi

if ! [ -f $elf_file ]
then
	echo "Error, Elf File not found"
	exit 1
fi


basepath=$(pwd)/arduino101_firmware/tools/scripts

if [ -f $basepath/symbol_footprint.sh ]
then
    sh $basepath/symbol_footprint.sh mem ./out $elf_file
else
    echo "symbol_footprint.sh script not found"
    exit 1
fi

if [ -f $basepath/symbol_mem_info.sh ]
then
    txt="sh "$basepath"/symbol_mem_info.sh "
else
    echo "symbol_mem_info.sh script not found"
    exit 1
fi

find . | egrep -v "git|repo|manifest|pub|out" > out/mod_mem_info.sh
sed  '1d' out/mod_mem_info.sh > out/mod_mem_info_tmp.sh
sed 's#./#'"$txt"'#' out/mod_mem_info_tmp.sh > out/mod_mem_info.sh

chmod 777 out/mod_mem_info.sh

echo "Module, rodata, text, bss, total without bss" >  out/$out_file
sh out/mod_mem_info.sh >> out/$out_file

rm out/mod_mem_info*
echo "output files: out/"$out_file

if [ $level -ne 0 ]
then

    find . -maxdepth $level | egrep -v "git|repo|manifest|pub|out" > out/mod_mem_info.sh
    sed  '1d' out/mod_mem_info.sh > out/mod_mem_info_tmp.sh
    sed 's#./#'"$txt"'#' out/mod_mem_info_tmp.sh > out/mod_mem_info.sh

    chmod 777 out/mod_mem_info.sh

    echo "Module, rodata, text, bss, total, total without bss" >  out/tmp_$out_file
    sh out/mod_mem_info.sh  >> out/tmp_$out_file

    rm out/mod_mem_info*

    sed  '/File:/d'  out/tmp_$out_file > out/high_level_$out_file

    echo "output files: out/"high_level_$out_file
fi







