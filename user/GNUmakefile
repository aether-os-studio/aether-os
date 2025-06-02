export SYSROOT := $(shell pwd)/rootfs-$(ARCH)/usr

all:
	mkdir -p $(SYSROOT)
	bash ports/ports.sh

clean:
