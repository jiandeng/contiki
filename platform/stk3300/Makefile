TARGET=stk3300

all: stk3300


CONTIKI=../..

CONTIKI_TARGET_MAIN=contiki-main.c

PROJECT_SOURCEFILES = parity.c 


randgen: randgen.c
	gcc -DNDEBUG -I $(CONTIKI)/cpu/arm/efm32/ -I . -I $(CONTIKI)/core  randgen.c -o randgen

randcheck: randcheck.c
	gcc -DNDEBUG -I $(CONTIKI)/cpu/arm/efm32/ -I . -I $(CONTIKI)/core  randcheck.c -o randcheck

clean: stk3300_clean

.PHONY: stk3300_clean

stk3300_clean:

include $(CONTIKI)/Makefile.include

