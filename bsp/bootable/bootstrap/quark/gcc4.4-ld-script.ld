OUTPUT_ARCH(i386)
OUTPUT_FORMAT("elf32-i386")

MEMORY
{
	rom (rw) : ORIGIN = 0xFFFFE400, LENGTH = 7k - 16
}
SECTIONS
{
  . = ORIGIN(rom);

   _text_base = .;

  .text :
  {
   *(.text.entry)
   *(.text)
   *(.text.last)
   *(.text.*)
  }

  /*Put data at the bottom of the 80K of RAM*/
  . = 0xA8000000;

  . = ALIGN(4);
  .rdata :
  {
   *(.rdata)
   *(.rdata.*)
  }

  . = ALIGN(4);
  /* Data load address is here, but appears in the image after .rdata */
  .data : AT ( ADDR (.rdata) + SIZEOF (.rdata) )
  {
   __data_lma = LOADADDR(.data); __data_vma = .;
   *(.data)
   *(.data.*)
   __data_size = SIZEOF( .data );
  }

  . = ALIGN(4);
  .bss :
  {
   __bss_start = .;
   *(.bss)
   *(COMMON)
   __bss_end = .;
  }

  . = ALIGN(4);
  .eh_frame :
  {
     *(.eh_frame)
  }

  . = ALIGN(4);
  .drectve :
  {
     *(.drectve)
  }

  .comment 0 : { *(.comment) }
}

