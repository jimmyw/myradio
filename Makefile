
all: adc.elf

CC=arm-none-eabi-gcc
LDFLAGS=-mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -ggdb3
CFLAGS= -Os -std=c99 \
			 -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -ggdb3 \
			 -fno-common -ffunction-sections \
			 -fdata-sections  -MD -DSTM32F1 -Ilibopencm3//include

clean:
	rm -rf adc.elf *.o

flash: adc.elf
	echo "halt; program $(abspath $<) verify reset" | nc -4 localhost 4444

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<
adc.elf: adc.o
	$(CC) --static -nostartfiles -Tadc.ld $(LDFLAGS) -Wl,-Map=adc.map -Wl,--cref -Wl,--gc-sections -Llibopencm3/lib -L. $^ -lopencm3_stm32f1 -Wl,--start-group u8g2.a -lc -lgcc -lnosys -Wl,--end-group -o adc.elf

openocd:
	openocd -f jlink.cfg
