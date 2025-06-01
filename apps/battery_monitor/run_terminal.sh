#!/bin/bash
export CARGO_TARGET_THUMBV7EM_NONE_EABI_RUNNER="probe-rs run --chip STM32G431CBUx --probe 0483:3748:39"
cargo build --release --features=terminal
cargo size --release --features=terminal
cargo run  --release --features=terminal
