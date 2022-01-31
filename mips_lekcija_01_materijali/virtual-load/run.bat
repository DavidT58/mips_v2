@rem asembliranje
arm-none-eabi-gcc.exe -c -mcpu=cortex-m3 -mthumb -o virtual-load.o virtual-load.s

@rem povezivanje
arm-none-eabi-ld.exe --script=linker-script.ld -o virtual-load.elf virtual-load.o

@rem formiranje hex fajla
arm-none-eabi-objcopy.exe -O ihex virtual-load.elf virtual-load.hex

@rem pregled sadrzaja elf fajla
arm-none-eabi-objdump.exe -thsd virtual-load.elf