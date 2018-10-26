SVCNAME = Tachikoma-Poller

EXTRAFLAGS = -fno-omit-frame-pointer -Wno-incompatible-pointer-types-discards-qualifiers -Wno-uninitialized -Wno-sometimes-uninitialized -ferror-limit=0 -Wno-unused-parameter -O0 -Wno-unused-result -Wno-ignored-qualifiers -mno-sse

CC = clang-3.6 
AS = clang-3.6
CFLAGS = -pie -g -Wall -Wshadow -Wextra -std=c99 $(EXTRAFLAGS) -m32 -fpic

# This makes it so that each c file is compiled in the directory
CFILES = $(wildcard *.c)
OBJECTS = $(patsubst %.c,%.o,$(CFILES))
AS_OBJECTS = $(patsubst %.S,%.o,$(wildcard *.S))
LIBS = -lm

$(SVCNAME): $(OBJECTS) $(AS_OBJECTS)
	$(CC) $(CFLAGS) $(DEFINES) $(OBJECTS) $(AS_OBJECTS) -o $(SVCNAME) $(LIBS)
	#strip -s $(SVCNAME)

%.o:  %.S
	$(AS) -m32 -c $< -o $@

%.o:  %.c Makefile
	$(CC) $(CFLAGS) $(DEFINES) -c $< -o $@

clean:
	rm -rf $(SVCNAME) *.o

.PHONY: clean
