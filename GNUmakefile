# Nuke built-in rules and variables.
MAKEFLAGS += -rR
.SUFFIXES:

SMP ?= 2
DEBUG ?= 0
KVM ?= 0
SUDO ?= 0

# Default user QEMU flags. These are appended to the QEMU command calls.
QEMUFLAGS := -M q35 -cpu max -m 4G -smp $(SMP)

ifeq ($(DEBUG), 1)
QEMUFLAGS += -s -S
endif

ifeq ($(KVM), 1)
QEMUFLAGS += --enable-kvm
endif

override IMAGE_NAME := aether

# Toolchain for building the 'limine' executable for the host.
HOST_CC := cc
HOST_CFLAGS := -g -O2 -pipe
HOST_CPPFLAGS :=
HOST_LDFLAGS :=
HOST_LIBS :=

.PHONY: all
all: $(IMAGE_NAME).iso

.PHONY: all-hdd
all-hdd: $(IMAGE_NAME).hdd

.PHONY: run
run: ovmf/ovmf-code-x86_64.fd $(IMAGE_NAME).iso
ifeq ($(SUDO), 1)
	sudo qemu-system-x86_64 \
		-M q35 \
		-drive if=pflash,unit=0,format=raw,file=ovmf/ovmf-code-x86_64.fd,readonly=on \
		-drive if=none,file=$(IMAGE_NAME).iso,format=raw,id=cdrom \
		-boot d \
		-device ahci,id=ahci \
		-device ide-cd,drive=cdrom,bus=ahci.0 \
		$(QEMUFLAGS)
else
	qemu-system-x86_64 \
		-M q35 \
		-drive if=pflash,unit=0,format=raw,file=ovmf/ovmf-code-x86_64.fd,readonly=on \
		-drive if=none,file=$(IMAGE_NAME).iso,format=raw,id=cdrom \
		-boot d \
		-device ahci,id=ahci \
		-device ide-cd,drive=cdrom,bus=ahci.0 \
		$(QEMUFLAGS)
endif

.PHONY: run-hdd
run-hdd: ovmf/ovmf-code-x86_64.fd $(IMAGE_NAME).hdd
ifeq ($(SUDO), 1)
	sudo qemu-system-x86_64 \
		-M q35 \
		-drive if=pflash,unit=0,format=raw,file=ovmf/ovmf-code-x86_64.fd,readonly=on \
		-drive if=none,file=$(IMAGE_NAME).hdd,format=raw,id=harddisk \
		-device ahci,id=ahci \
		-device nvme,drive=harddisk,serial=1234 \
		$(QEMUFLAGS)
else
	qemu-system-x86_64 \
		-M q35 \
		-drive if=pflash,unit=0,format=raw,file=ovmf/ovmf-code-x86_64.fd,readonly=on \
		-drive if=none,file=$(IMAGE_NAME).hdd,format=raw,id=harddisk \
		-device ahci,id=ahci \
		-device nvme,drive=harddisk,serial=1234 \
		$(QEMUFLAGS)
endif

ovmf/ovmf-code-x86_64.fd:
	mkdir -p ovmf
	curl -Lo $@ https://github.com/osdev0/edk2-ovmf-nightly/releases/latest/download/ovmf-code-x86_64.fd

limine/limine:
	rm -rf limine
	git clone https://github.com/limine-bootloader/limine.git --branch=v9.x-binary --depth=1
	$(MAKE) -C limine \
		CC="$(HOST_CC)" \
		CFLAGS="$(HOST_CFLAGS)" \
		CPPFLAGS="$(HOST_CPPFLAGS)" \
		LDFLAGS="$(HOST_LDFLAGS)" \
		LIBS="$(HOST_LIBS)"

kernel-deps:
	./kernel/get-deps
	touch kernel-deps

.PHONY: kernel
kernel: kernel-deps
	$(MAKE) -C kernel

.PHONY: user
user:
	$(MAKE) -C user

$(IMAGE_NAME).iso: limine/limine kernel user
	rm -rf iso_root
	mkdir -p iso_root/boot
	cp -v kernel/bin/kernel iso_root/boot/
	mkdir -p iso_root/boot/limine
	cp -v limine.conf limine/limine-bios.sys limine/limine-bios-cd.bin limine/limine-uefi-cd.bin iso_root/boot/limine/
	mkdir -p iso_root/EFI/BOOT
	cp -v limine/BOOTX64.EFI iso_root/EFI/BOOT/
	cp -v limine/BOOTIA32.EFI iso_root/EFI/BOOT/
	mkdir -p iso_root/usr/bin
	cp -v user/init/initd.exec iso_root/usr/bin
	cp -v user/acpid/acpid.exec iso_root/usr/bin
	cp -v user/pcid/pcid.exec iso_root/usr/bin
	cp -v user/blkd/blkd.exec iso_root/usr/bin
	cp -v user/ps2d/ps2d.exec iso_root/usr/bin
	cp -v user/ahcid/ahcid.exec iso_root/usr/bin
	cp -v user/nvmed/nvmed.exec iso_root/usr/bin
	cp -v user/fsd/fsd.exec iso_root/usr/bin
	cp -v user/shell/shell.exec iso_root/usr/bin
	xorriso -as mkisofs -R -r -J -b boot/limine/limine-bios-cd.bin \
		-no-emul-boot -boot-load-size 4 -boot-info-table -hfsplus \
		-apm-block-size 2048 --efi-boot boot/limine/limine-uefi-cd.bin \
		-efi-boot-part --efi-boot-image --protective-msdos-label \
		iso_root -o $(IMAGE_NAME).iso
	./limine/limine bios-install $(IMAGE_NAME).iso
	rm -rf iso_root

$(IMAGE_NAME).hdd: limine/limine kernel user
	rm -rf $(IMAGE_NAME).hdd
	dd if=/dev/zero of=$(IMAGE_NAME).hdd bs=1M count=64
	sgdisk $(IMAGE_NAME).hdd -n 1:2048 -t 1:ef00
	mformat -i $(IMAGE_NAME).hdd@@1M
	mmd -i $(IMAGE_NAME).hdd@@1M ::/EFI ::/EFI/BOOT ::/boot ::/boot/limine ::/usr ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M kernel/bin/kernel ::/boot
	mcopy -i $(IMAGE_NAME).hdd@@1M limine.conf limine/limine-bios.sys ::/boot/limine
	mcopy -i $(IMAGE_NAME).hdd@@1M limine/BOOTX64.EFI ::/EFI/BOOT
	mcopy -i $(IMAGE_NAME).hdd@@1M limine/BOOTIA32.EFI ::/EFI/BOOT
	mcopy -i $(IMAGE_NAME).hdd@@1M user/init/initd.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/acpid/acpid.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/pcid/pcid.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/blkd/blkd.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/ps2d/ps2d.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/ahcid/ahcid.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/nvmed/nvmed.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/fsd/fsd.exec ::/usr/bin
	mcopy -i $(IMAGE_NAME).hdd@@1M user/shell/shell.exec ::/usr/bin

	qemu-img convert -O vmdk $(IMAGE_NAME).hdd $(IMAGE_NAME).vmdk

.PHONY: clean
clean:
	$(MAKE) -C kernel clean
	$(MAKE) -C user clean
	rm -rf iso_root $(IMAGE_NAME).iso $(IMAGE_NAME).hdd $(IMAGE_NAME).vmdk

.PHONY: distclean
distclean: clean
	$(MAKE) -C kernel distclean
	$(MAKE) -C user clean
	rm -rf kernel-deps limine ovmf
