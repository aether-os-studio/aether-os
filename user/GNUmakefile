export SYSROOT := $(shell pwd)/rootfs-$(ARCH)/usr

all:
	mkdir -p $(SYSROOT)/bin
	$(MAKE) -C init
	bash ports/ports.sh

clean:
	$(MAKE) -C init clean
