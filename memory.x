MEMORY
{
    /* The RP2040 has 2MB of external flash. */
    /* The first 256 bytes are for the bootloader. */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

SECTIONS
{
    .boot2 : ALIGN(4)
    {
        __boot2_start__ = .;
        KEEP(*(.boot2))
        __boot2_end__ = .;
    } > BOOT2
} INSERT BEFORE .text;
