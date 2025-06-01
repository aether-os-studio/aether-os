all:
	RUSTFLAGS="-C relocation-model=static" cargo build --target $(ARCH)-unknown-linux-musl
	mkdir -p ../rootfs-$(ARCH)/bin/
	cp -r target/$(ARCH)-unknown-linux-musl/debug/init ../rootfs-$(ARCH)/bin/

clean:
	cargo clean
