CC ?= gcc
CFLAGS ?= -O2 -std=c99 -Wall -Wextra -Wno-missing-braces -I..
digix0x-host: main.c ../tb303_core.c ../tb303_core.h ../config.h
	$(CC) $(CFLAGS) -o $@ main.c ../tb303_core.c -lm
test: digix0x-host
	./digix0x-host
clean:
	rm -f digix0x-host test_sequence.wav test_midi.wav
.PHONY: test clean
