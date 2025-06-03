all:
	cargo build --target=x86_64-unknown-linux-musl
	cp target/x86_64-unknown-linux-musl/debug/init $(SYSROOT)/bin/

clean:
	cargo clean
