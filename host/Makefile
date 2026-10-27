CC ?= gcc
CFLAGS ?= -O2 -std=c99 -Wall -Wextra -Wno-missing-braces -I..
digix0x-host: main.c ../tb303_core.c ../tb303_core.h ../config.h
	$(CC) $(CFLAGS) -o $@ main.c ../tb303_core.c -lm
sim: sim.c ../tb303_core.c ../tb303_core.h ../config.h
	clang -O2 -std=c99 -Wall -Wextra -Wno-missing-braces -I.. -o $@ sim.c ../tb303_core.c -lm -framework AudioToolbox -framework CoreMIDI -framework CoreFoundation
simtest: sim
	./sim --selftest
bench: bench.c ../tb303_core.c ../tb303_core.h ../config.h
	$(CC) -O3 -std=c99 -Wall -Wextra -Wno-missing-braces -I.. -o $@ bench.c ../tb303_core.c -lm
	./bench
vco: vco-render
	./vco-render
vco-render: vco_main.c ../tb303_core.c ../tb303_core.h ../config.h
	$(CC) $(CFLAGS) -o $@ vco_main.c ../tb303_core.c -lm
validate: vco
	../.venv/bin/python validate_vco.py
test: digix0x-host test-tuning
	./digix0x-host
	./test-tuning
test-tuning: test_tuning.c ../tb303_core.c ../tb303_core.h ../config.h
	$(CC) $(CFLAGS) -o $@ test_tuning.c ../tb303_core.c -lm
clean:
	rm -f digix0x-host vco-render test-tuning test_sequence.wav test_midi.wav
	rm -rf vco_out
.PHONY: test clean vco validate
