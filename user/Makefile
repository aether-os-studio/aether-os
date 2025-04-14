export CWD = $(shell pwd)
export CFLAGS = -static -nostdinc -nostdlib -mno-red-zone -fno-builtin -fno-stack-protector -g -O0 -I$(CWD)/alibc/include

all:
	$(MAKE) -C alibc

	$(MAKE) -C init
	$(MAKE) -C acpid
	$(MAKE) -C pcid

clean:
	rm -rf obj
