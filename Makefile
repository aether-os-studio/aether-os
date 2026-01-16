export ARCH ?= x86_64
export RUST_PROFILE ?= dev

export DEBUG ?= 0
export SMP ?= 2

all:
	$(MAKE) -C user
	$(MAKE) -C kernel

clippy:
	$(MAKE) -C kernel clippy

run: all
	$(MAKE) -C kernel run
