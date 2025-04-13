export CWD = $(shell pwd)
export CFLAGS = -static -nostdinc -nostdlib -mno-red-zone -I$(CWD)/alibc/include

all:
	$(MAKE) -C alibc

	$(MAKE) -C init
	$(MAKE) -C acpid

clean:
	rm -rf obj
