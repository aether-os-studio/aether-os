fn main() {
    println!("Hello world!!!");

    let pid = unsafe { libc::fork() };
    if pid == 0 {
        println!("Child process");
    } else {
        println!("Parent process");
    }

    loop {
        core::hint::spin_loop();
    }
}
