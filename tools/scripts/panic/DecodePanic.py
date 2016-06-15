#!/usr/bin/python
"""
Script used to parse panic data
"""

import os
import sys
import struct
import subprocess
import linecache
import DecodeRegister

class DecodePanic(object):
    """
    decode panic raw data thanks to the architecture
    """

    fields_ARCH_BASE_START = [
      ("magic", "L"),
      ("struct_size", "L"),
      ("build_cksum", "L"),
      ("time", "L"),
      ("arch", "B"),
      ("struct_version", "B"),
      ("flags", "B"),
      ("reserved", "B")]

    fields_ARCH_DEFAULT = [
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),

      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),

      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L"),
      ("", "L")]

    fields_ARCH_CORTEX_M = [
      ("dfsr", "L"),
      ("hfsr", "L"),
      ("shcsr", "L"),
      ("mfar", "L"),
      ("bfar", "L"),
      ("mmfs", "L"),

      ("xPSR", "L"),
      ("pc", "L"),
      ("lr", "L"),
      ("r12", "L"),
      ("r3", "L"),
      ("r2", "L"),
      ("r1", "L"),
      ("r0", "L"),

      ("lr", "L"),
      ("r11", "L"),
      ("r10", "L"),
      ("r9", "L"),
      ("r8", "L"),
      ("r7", "L"),
      ("r6", "L"),
      ("r5", "L"),
      ("r4", "L"),
      ("msp", "L"),
      ("ipsr", "L"),
      ("psp", "L")]

    fields_ARCH_ARCV2 = [
      ("ecr", "L"),
      ("efa", "L"),
      ("eret", "L"),
      ("erstatus", "L"),

      ("r0", "L"),
      ("r1", "L"),
      ("r2", "L"),
      ("r3", "L"),
      ("r4", "L"),
      ("r5", "L"),
      ("r6", "L"),
      ("r7", "L"),
      ("r8", "L"),
      ("r9", "L"),
      ("r10", "L"),
      ("r11", "L"),
      ("r12", "L"),
      ("r13", "L"),
      ("r14", "L"),
      ("r15", "L"),
      ("r16", "L"),
      ("r17", "L"),
      ("r18", "L"),
      ("r19", "L"),
      ("r20", "L"),
      ("r21", "L"),
      ("r22", "L"),
      ("r23", "L"),
      ("r24", "L"),
      ("r25", "L"),

      ("gp", "L"),
      ("fp", "L"),
      ("sp", "L"),
      ("il", "L"),
      ("r30", "L"),
      ("bl", "L")]

    fields_ARCH_X86 = [
      # Panic type (not a register)
      ("type", "L"),
      ("cr2", "L"),

      # Callee saved registers
      ("ebp", "L"),
      ("ebx", "L"),
      ("esi", "L"),
      ("edi", "L"),

      # Stack registers
      ("esp", "L"),
      ("ss", "L"),

      # Caller saved registers
      ("edx", "L"),
      ("ecx", "L"),
      ("eax", "L"),

      #  Data pushed by cpu during exception
      ("error", "L"),
      ("eip", "L"),
      ("cs", "L"),
      ("flags", "L"),
      ("priv_esp", "L"),
      ("priv_ss", "L")
    ]

    # correspondance between "arch" field from raw panic data and parsing table
    # TODO: this has to be consistent with the c code, which is not the case between projects
    # These values are valid for curie only
    arch_table = [
                  fields_ARCH_X86,
                  fields_ARCH_ARCV2,
                  fields_ARCH_CORTEX_M
                 ]

    def __init__(self, elf_path, mem):
        if elf_path != None and os.path.isdir(elf_path):
            self.elf_path = elf_path
        else:
            self.elf_path = None
        self.mem = mem
        assert(self.mem[:4] == "Pnc!")

    def __str__(self):
        def parse(fields, data):
            fmt = "<" + "".join([x[1] for x in fields])
            size = struct.calcsize(fmt)
            return struct.unpack(fmt, data[:size])

        # extract the beginning of the panic
        (magic, struct_size, build_cksum, time, arch, struct_version, flags, reserved) \
            = parse(self.fields_ARCH_BASE_START, self.mem)
        assert(magic == 0x21636e50)

        # set fields table
        arch_fields = self.arch_table[arch]
        fmt = "<" + "".join([x[1] for x in arch_fields])
        arch_size = struct.calcsize(fmt)

        fmt = "<" + "".join([x[1] for x in self.fields_ARCH_BASE_START])
        header_size = struct.calcsize(fmt)

        fields_stack = [("stack%d" % i, "L") for i in range(0, (struct_size-arch_size-header_size)/4)]
        if len(fields_stack)==0:
            fields = self.fields_ARCH_BASE_START + arch_fields
        else:
            fields = self.fields_ARCH_BASE_START + fields_stack + arch_fields
        unpacked_mem = parse(fields, self.mem)

        output_string = ""
        for (idx, field) in enumerate(fields):
            if field[0] != "":
                output_string += ("%s = 0x%x\n"%(field[0], unpacked_mem[idx]))

            if field[0] == "ipsr" and arch == 2:
                output_string += ("%s\n" %(DecodeRegister.decode_nrf51_ipsr(unpacked_mem[idx])))

            if field[0] == "xPSR" and arch == 2:
                output_string += ("%s\n" %(DecodeRegister.decode_nrf51_xpsr(unpacked_mem[idx])))

            if field[0] in ("lr", "pc", "eip", "sp", "eret") and self.elf_path != None:
                for file in os.listdir(self.elf_path):
                    if file.endswith(".elf"):
                        file = os.path.join(self.elf_path, file)
                        addr2line_string = subprocess.check_output(("addr2line -f -p -i -e "+file+" "+hex(unpacked_mem[idx])), shell=True)
                        if "??:0" not in addr2line_string:
                            fault_line = self.__get_fault_line(addr2line_string.split("\n")[0].split("(")[0], field[0])
                            output_string += ("\t---- addr2line for 0x%x ---- \n"%(unpacked_mem[idx]))
                            output_string += "\t%s" %(file)
                            output_string += "\n\t"
                            output_string += "\t%s" %(addr2line_string)
                            output_string += "\n"
                            output_string += fault_line
                            output_string += "\n"
                            output_string += ("\t------------------------------- \n")
        return output_string

    @classmethod
    def __get_fault_line(cls, file_name_and_line, field):
        """
        print fault line in source code
        """
        try:
            file_name = file_name_and_line[file_name_and_line.find("/"):].split(":")[0]
            file_line = int(file_name_and_line[file_name_and_line.find("/"):].split(":")[1])
            if field in ("lr", "eret"):
                return linecache.getline(file_name, file_line - 2) + "\t\t==\n" + linecache.getline(file_name, file_line - 1) + "\t\t==\n" + linecache.getline(file_name, file_line)
            if field in ("pc", "eip", "sp"):
                return linecache.getline(file_name, file_line - 1) + "\t\t==\n" + linecache.getline(file_name, file_line) + "\t\t==\n" + linecache.getline(file_name, file_line + 1)

        except Exception:
            return ""

def SplitPanics(data):
    """
    Function to split a panic memory dump in different panics
    """
    index = 0
    panics = []
    while index < len(data):
        if data[index:index+4] == "Pnc!":
            length = struct.unpack("<L", data[index+4:index+8])[0]
            panics.append(data[index:index+length])
            index += length
        else:
            index += 4
    return panics

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description='''Decode a panic partition data from a string or a file''',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog='''The panic content can be a file or raw formatted data
        (e.g.
        506E632174000000FF7F31126CFE41800010FB6F2000FFFF0008F7DE0008FB9F00000020
        F1CD0100F3CD010008280020882E0020020000001300000008000000F1FFFFFF00000000
        0027FF1FFFFFFFFFFFFFFFFF0C29002004290020242B0020240100005826002000000000
        FCFFFFFF01020100)''')

    parser.add_argument('elf_path', action='store',help='Path to the ELF file')
    parser.add_argument('panic_content', action='store',help='Panic memory content, can be a file or raw data')
    args = parser.parse_args(sys.argv[1:])

    if os.path.isfile(args.panic_content):
        with open(args.panic_content, 'r') as content_file:
            panic_partition = content_file.read()
        panics = SplitPanics(panic_partition)
    else:
        panics = [''.join([chr(int(X, 16)) for X in [args.panic_content[start:start+2] for start in range(0, len(args.panic_content), 2)]])]

    for panic_memory in panics:
        panic_data = DecodePanic(args.elf_path, panic_memory)
        print panic_data
