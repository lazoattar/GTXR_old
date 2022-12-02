/* This will work with STM32 type of microcontrollers.    *
 * The sizes of RAM and flash are specified smaller than  *
 * what most of the STM32 provide to ensure that the demo *
 * program will run on ANY STM32.                         */

 _Min_Heap_Size = 0x200 ;      /* required amount of heap  */
_Min_Stack_Size = 0x400 ; /* required amount of stack */

MEMORY
{
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
}

SECTIONS
{
      .text :
    {
        *(vectors)    /* Vector table */
        *(.text)      /* Program code */
        *(.rodata)    /* Read only data */
    } >rom


    .data :
    {
        *(.data)      /* Data memory */
    } >ram

    .bss :
    {
        *(.bss)       /* Zero-filled run time allocate data memory */
    } >ram

    /* User_heap_stack section, used to check that there is enough RAM left */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >DTCMRAM
}
