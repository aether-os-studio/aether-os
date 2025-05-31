all:
	cargo build --target x86_64-unknown-none
	mkdir -p ../rootfs-x86_64/bin/
	cp -r target/x86_64-unknown-none/debug/init ../rootfs-x86_64/bin/

clean:
	cargo clean
