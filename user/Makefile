export CWD = $(shell pwd)
export CFLAGS = -static -nostdinc -nostdlib -mno-red-zone -fno-builtin -fno-stack-protector -g -O0 -I$(CWD)/alibc/include

all:
	$(MAKE) -C alibc

	$(MAKE) -C init
	$(MAKE) -C acpid
	$(MAKE) -C pcid
	$(MAKE) -C blkd
	$(MAKE) -C ps2d
	$(MAKE) -C ahcid
	$(MAKE) -C nvmed
	$(MAKE) -C fsd

	$(MAKE) -C shell

clean:
	rm -rf $(shell find . -name "*.o") $(shell find . -name "*.exec") obj/
