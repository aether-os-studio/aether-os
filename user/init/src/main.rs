fn main() {
    println!("Hello world!!!");

    loop {
        core::hint::spin_loop();
    }
}
