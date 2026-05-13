
# Requirements

Port battery monitor to embassy.

Currently there are 2 build targets, one for the sensor, one for the terminal. They can be built with these 2 commands:
- cargo build --release --features=power_sensors
- cargo build --release --features=terminal

Follow a step-by-step process, 

Avoid making changes that won't be strictly required for Embassy port. Add any good ideas to a section in this working document.

Current implementation already heavily uses parts of embassy, just not the executor. It uses embassy-stm32 as the main HAL, also uses some other embassy crates.

## Phase 0

Do a review of the codebase, in particular bsp.rs and main.rs to see what RTIC sync primatives need replacing. Incorporate this into the Phase 1 plan.

Also determine the specifics of Phase 1 and incorporate into the Phase 1 plan.

## Phase 1

Try to prepare as much as possible before actually making the switch. Keep both build targets compiling. At each step, test that both targets still build. I use: cargo build --release --features=power_sensors && cargo build --release --features=terminal
- Test that both targets build
- Port as much as we can to using plain async tasks.
- Start porting existing functionality to use global static's like they will need for embassy with the required synchronisation constructs
- Any other baby steps.
- Pull anything we can out of the RTIC Shared/Local
- Add embassy executor to the Cargo.toml

## Phase 2

Only when we are sure we have made as many pre-changes, should we do the final port, removing RTIC and replacing with embassy-executor. 

Need user to confirm before moving to this task.

Both targets should build before starting this task with:
cargo build --release --features=power_sensors && cargo build --release --features=terminal

## Future work

- High priority executor.

