export ARCH ?= x86_64

SOURCE ?= $(shell pwd)
LD_SCRIPT=$(SOURCE)/linkers/$(ARCH).ld

RUST_TARGET ?= $(ARCH)-unknown-none
RUST_PROFILE ?= dev
ifeq ($(RUST_PROFILE), dev)
RUST_PROFILE_SUBDIR := debug
else
RUST_PROFILE_SUBDIR := release
endif

LOCKFILE := $(SOURCE)/Cargo.lock
MANIFEST := $(SOURCE)/Cargo.toml

export BUILD = $(SOURCE)/build

DEBUG ?= 0
SMP ?= 2

QEMUFLAGS ?= -m 4G -serial stdio
ifneq ($(DEBUG), 0)
QEMUFLAGS += -s -S
endif

.PHONY: $(BUILD)/kernel-$(ARCH).elf
$(BUILD)/kernel-$(ARCH).elf:
	mkdir -p $(BUILD)
	RUSTFLAGS="-C relocation-model=static" cargo build --profile $(RUST_PROFILE) --target $(RUST_TARGET) -Z build-std=core,alloc -Zbuild-std-features=compiler-builtins-mem
	cp target/$(RUST_TARGET)/$(RUST_PROFILE_SUBDIR)/kernel $(BUILD)/kernel-$(ARCH).elf

limine/limine:
	rm -rf limine
	git clone https://github.com/limine-bootloader/limine.git --branch=v10.x-binary --depth=1
	$(MAKE) -C limine

aether-$(ARCH).img: limine/limine $(BUILD)/kernel-$(ARCH).elf
	rm -f aether-$(ARCH).img
	dd if=/dev/zero bs=1M count=0 seek=64 of=aether-$(ARCH).img
	sgdisk aether-$(ARCH).img -n 1:2048 -t 1:ef00
	mformat -i aether-$(ARCH).img@@1M
	mmd -i aether-$(ARCH).img@@1M ::/EFI ::/EFI/BOOT ::/boot ::/boot/limine
	mcopy -i aether-$(ARCH).img@@1M $(BUILD)/kernel-$(ARCH).elf ::/boot/kernel.elf
	mcopy -i aether-$(ARCH).img@@1M limine.conf ::/boot/limine
ifeq ($(ARCH),x86_64)
	mcopy -i aether-$(ARCH).img@@1M limine/limine-bios.sys ::/boot/limine
	mcopy -i aether-$(ARCH).img@@1M limine/BOOTX64.EFI ::/EFI/BOOT
	mcopy -i aether-$(ARCH).img@@1M limine/BOOTIA32.EFI ::/EFI/BOOT
endif
ifeq ($(ARCH),aarch64)
	mcopy -i aether-$(ARCH).img@@1M limine/BOOTAA64.EFI ::/EFI/BOOT
endif
ifeq ($(ARCH),riscv64)
	mcopy -i aether-$(ARCH).img@@1M limine/BOOTRISCV64.EFI ::/EFI/BOOT
endif
ifeq ($(ARCH),loongarch64)
	mcopy -i aether-$(ARCH).img@@1M limine/BOOTLOONGARCH64.EFI ::/EFI/BOOT
endif

edk2-ovmf:
	curl -L https://github.com/osdev0/edk2-ovmf-nightly/releases/download/nightly-20260105T013525Z/edk2-ovmf.tar.gz | gunzip | tar -xf -

run: run-$(ARCH)

run-x86_64: edk2-ovmf aether-$(ARCH).img
	qemu-system-x86_64 \
		-M q35 \
		-smp $(SMP) \
		-cpu max \
		-device qemu-xhci \
		-device usb-kbd \
		-device usb-mouse \
		-drive if=pflash,unit=0,format=raw,file=edk2-ovmf/ovmf-code-$(ARCH).fd,readonly=on \
		-hda aether-$(ARCH).img \
		$(QEMUFLAGS)

run-aarch64: edk2-ovmf aether-$(ARCH).img
	qemu-system-aarch64 \
		-M virt \
		-smp $(SMP) \
		-cpu cortex-a72 \
		-device ramfb \
		-device qemu-xhci \
		-device usb-kbd \
		-device usb-mouse \
		-drive if=pflash,unit=0,format=raw,file=edk2-ovmf/ovmf-code-$(ARCH).fd,readonly=on \
		-hda aether-$(ARCH).img \
		$(QEMUFLAGS)
