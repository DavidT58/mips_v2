@rem pretprocesiranje
arm-none-eabi-gcc.exe -E -o main.i main.c

@rem prevodjenje
arm-none-eabi-gcc.exe -S -mcpu=cortex-m3 -mthumb -o main.s main.i

@rem asembliranje
arm-none-eabi-gcc.exe -c -mcpu=cortex-m3 -mthumb -o main.o main.s

@rem povezivanje
arm-none-eabi-ld.exe --script=linker-script.ld -o main.elf main.o

@rem formiranje hex fajla
arm-none-eabi-objcopy.exe -O ihex main.elf main.hex

@rem pregled sadrzaja elf fajla
arm-none-eabi-objdump.exe -thsd main.elf