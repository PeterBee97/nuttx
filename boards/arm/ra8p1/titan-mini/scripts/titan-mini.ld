/* SPDX-License-Identifier: Apache-2.0 */

/* CPU0 secure aliases. Keep all allocations in the first 448 KiB of SRAM.
 * No option-setting memory, external SDRAM or CPU1 memory is programmed.
 */

MEMORY
{
  mram (rx)  : ORIGIN = 0x02000000, LENGTH = 1M
  sram (rwx) : ORIGIN = 0x22000000, LENGTH = 448K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(__start)

SECTIONS
{
  .text :
  {
    _stext = .;
    KEEP(*(.vectors))
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.glue_7 .glue_7t .got .gcc_except_table)
    . = ALIGN(4);
    _etext = .;
  } > mram

  .init_section :
  {
    _sinit = .;
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
    KEEP(*(.init_array .ctors))
    _einit = .;
  } > mram

  .ARM.extab : { *(.ARM.extab*) } > mram
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > mram

  .data :
  {
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > sram AT > mram
  _eronly = LOADADDR(.data);

  .bss (NOLOAD) :
  {
    . = ALIGN(32);
    _sbss = .;
    *(.bss .bss.* COMMON)
    . = ALIGN(32);
    _ebss = .;
  } > sram
}
