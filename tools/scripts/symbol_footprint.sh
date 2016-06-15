#!/bin/sh

if [ $# -eq 3 ];
then
	file_description=$1
	output_path=$2
	elf_file=$3
else
	echo "usage: $0 [file_description] [output_path] [elf_file]"
	exit 1
fi

nm -l -r --print-size --size-sort -td $elf_file |  awk 'BEGIN {IGNORECASE = 1; OFS = ","}
$3 == "b" {print $2, $4, $5} ' 2>&1 > ${output_path}/${file_description}_bss_symbols.csv
nm -l -r --print-size --size-sort -td $elf_file |  awk 'BEGIN {IGNORECASE = 1; OFS = ","}
$3 == "t" {print $2, $4, $5}' 2>&1 > ${output_path}/${file_description}_text_symbols.csv
nm -l -r --print-size --size-sort -td $elf_file |  awk 'BEGIN {IGNORECASE = 1; OFS = ","}
$3 == "r" {print $2, $4, $5}' 2>&1 > ${output_path}/${file_description}_rodata_symbols.csv

