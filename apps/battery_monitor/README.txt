

To run:

cargo run --bin battery_monitor --release

Reset programmer??:
cargo run -- --connect-under-reset

To use GDB:
cargo embed
gdb-multiarch target/thumbv7m-none-eabi/debug/battery_monitor
target remote :1337

rust-size ./target/thumbv7m-none-eabi/release/battery_monitor


Links:
https://probe.rs/docs/tools/vscode/
https://github.com/rtic-rs/rtic-examples/blob/master/
https://docs.rust-embedded.org/discovery/microbit/index.html
https://doc.rust-lang.org/rust-by-example/index.html
https://rtic.rs/1/book/en/
https://docs.rs/stm32f1xx-hal/0.10.0/stm32f1xx_hal/

