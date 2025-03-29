// use std::io::Read;

// fn read_char() -> char {
//     let mut buf: [u8; 1] = [0];
//     loop {
//         match std::io::stdin().read(&mut buf) {
//             Ok(_) => return buf[0] as char,
//             Err(e) => println!("read char failed: {}", e),
//         }
//     }
// }

// fn parse() -> ! {
//     loop {
//         let key = read_char();
//         print!("{}", key);
//     }
// }

fn main() {
    println!("init process is running, pid = {}", std::process::id());

    // panic!("Test for backtrace");
}
