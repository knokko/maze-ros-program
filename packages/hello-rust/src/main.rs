use std::io::Read;
use std::process::{Command, Stdio};

fn main() {
    println!("Hello Rust!");
    return;
    let mut left_wheel = Command::new("rostopic")
        .args(["echo", "/db4/wheels_driver_node/wheels_cmd/vel_left"])
        .stdout(Stdio::piped())
        .spawn().expect("rostopic should work");

    let output_stream = left_wheel.stdout.as_mut().expect("Should have output");
    let mut output_buffer = [0; 100];

    loop {
        let num_read_bytes = output_stream.read(&mut output_buffer).expect("Failed to read");
        if num_read_bytes == 0 { continue; }

        let string_result = String::from_utf8_lossy(&output_buffer[0 .. num_read_bytes]);
        println!("string result is {}", string_result);
    }
    println!("Finished Rust");
}
