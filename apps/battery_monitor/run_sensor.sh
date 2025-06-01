#!/bin/bash
export CARGO_TARGET_THUMBV7EM_NONE_EABI_RUNNER="probe-rs run --chip STM32G431CBUx --probe 0483:3748:52C3BF6C0648C2835248756225C287"
cargo build --release --features=power_sensors
cargo size --release --features=power_sensors
cargo run  --release --features=power_sensors
