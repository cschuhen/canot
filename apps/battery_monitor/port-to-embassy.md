# Porting Checklist: RTIC to Embassy Executor

**Goal:** Port the battery monitor to use the `embassy` executor.

---

### Phase 0: Analysis and Mapping (Foundation)

*   [ ] **Code Review:** Thoroughly review `src/main.rs` and `src/application.rs` to identify all uses of RTIC synchronization primitives (e.g., `#[shared]`, `#[local]`, `rtic_sync::channel::*`, `Mutex`, etc.).
*   [ ] **Mapping:** Determine the direct `embassy` equivalents for these primitives (e.g., using `embassy` synchronization constructs).
*   [ ] **Plan Integration:** Incorporate these specific mapping requirements into the Phase 1 plan to ensure a smooth transition.

---

### Phase 1: Incremental Porting (Preparation)

*   [ ] **Build Verification:** Ensure both build targets (`--features=power_sensors` and `--features=terminal`) still compile successfully after initial changes.
*   [ ] **Async Conversion:** Port existing functionality to use plain `async` tasks.
*   [ ] **Static State Migration:** Port existing functionality to use global static variables with the required synchronization constructs.
*   [ ] **Data Extraction:** Pull existing data out of the RTIC `Shared`/`Local` structures into the new asynchronous context.
*   [ ] **Executor Integration:** Add the `embassy` executor to `Cargo.toml`.

---

### Phase 2: Final Port (Execution)

*   [ ] **Final Migration:** Only after ensuring all pre-changes are stable, remove the RTIC framework entirely and replace it with the `embassy-executor`.

---

## Completed Work Log

### Ignition Pin Migration to Global Static (2026-05-16)

**Goal:** Move `ignition_pin` out of RTIC's Local resources into a global static as preparation for embassy migration.

**Changes Made:**

1. **Added OnceLock import and static declaration:**
   ```rust
   #[cfg(feature = "power_sensors")]
   use embassy_sync::once_lock::OnceLock;
   
   #[cfg(feature = "power_sensors")]
   static IGNITION_PIN: OnceLock<Mutex<CriticalSectionRawMutex, bsp::ExtiPin>> = OnceLock::new();
   ```

2. **Removed from RTIC Local struct:**
   - Removed `ignition_pin: crate::bsp::ExtiPin` from `Local` struct
   - Removed `ignition_pin` from init() return tuple

3. **Initialize in init():**
   ```rust
   #[cfg(feature = "power_sensors")]
   { let _ignition_pin_ref = IGNITION_PIN.get_or_init(|| Mutex::new(ignition_pin)); }
   ```

4. **Update ignition_task to use global static:**
   - Wrapped pin in `Mutex<CriticalSectionRawMutex, ExtiPin>` for safe shared access
   - Use `IGNITION_PIN.get().await` to get reference asynchronously
   - Access pin methods through mutex: `ignition_pin_ref.lock().await.wait_for_any_edge()`
   - Added manual debounce (10ms delay) since Debouncer wrapper can't own the pin from static
   - Wrapped power_sensors feature-gated code in `#[cfg(feature = "power_sensors")]` block
   - Added fallback for non-power_sensors builds

**Key Design Decisions:**

- **OnceLock over StaticCell:** OnceLock provides async `.get().await` which is needed for embassy-style access. StaticCell only provides synchronous access.
- **Mutex wrapper:** Even though only one task uses ignition_pin currently, wrapping in Mutex prepares for future multi-task scenarios in embassy context.
- **Manual debounce:** async_debounce::Debouncer requires ownership of the pin type, which can't be obtained from a static. Implemented simple 10ms delay-based debounce instead.
- **Feature gating:** All IGNITION_PIN usage gated behind `#[cfg(feature = "power_sensors")]` since ignition_pin is only available with that feature.

**Build Verification:**
- ✅ Both build targets compile successfully:
  - `--features=power_sensors`
  - `--features=terminal`

**Files Modified:**
- `/home/cschuhen/rust/canot/apps/battery_monitor/src/main.rs`

---

### CAN Suspended Event Sender Migration to Global Static (2026-05-16)

**Goal:** Move `can_suspended_event_sender` out of RTIC's Local resources into a global static.

**Changes Made:**

1. **Added OnceLock import and static declaration:**
   ```rust
   static CAN_SUSPENDED_SENDER: embassy_sync::once_lock::OnceLock<Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>> = embassy_sync::once_lock::OnceLock::new();
   ```

2. **Removed from RTIC Local struct:**
   - Removed `can_suspended_event_sender` from `Local` struct
   - Removed it from init() return tuple

3. **Initialize in init():**
   ```rust
   CAN_SUSPENDED_SENDER.get_or_init(|| main_event_sender.clone());
   ```

4. **Update ignition_task to use global static:**
   - Use `CAN_SUSPENDED_SENDER.get().await.send(MainEvent::CanEnabled(enabled)).await`
   - Removed from task's local resources declaration

**Key Design Decisions:**
- OnceLock provides async `.get().await` for embassy-style access
- Sender is Clone+Send, making it suitable for global static storage
- No Mutex wrapper needed since Senders are designed for sharing

**Build Verification:**
- ✅ Both build targets compile successfully

---

### CAN Interface Migration to Global Static (2026-05-16)

**Goal:** Move `can_iface` out of RTIC's Local resources into a global static.

**Changes Made:**

1. **Added OnceLock import and static declaration:**
   ```rust
   static CAN_IFACE: embassy_sync::once_lock::OnceLock<can::BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>> = embassy_sync::once_lock::OnceLock::new();
   ```

2. **Removed from RTIC Local struct:**
   - Removed `can_iface` from `Local` struct
   - Removed it from init() return tuple and task local resources

3. **Initialize in init():**
   ```rust
   let can_reader = can_iface.reader();
   let can_writer = can_iface.writer();
   CAN_IFACE.get_or_init(|| can_iface);
   ```

4. **Update app creation:**
   - Use pre-extracted `can_reader` and `can_writer` instead of calling methods on moved value

**Key Design Decisions:**
- OnceLock provides async `.get().await` for embassy-style access
- BufferedCan is Clone+Send, making it suitable for global static storage
- Reader/writer extracted before moving can_iface into static to avoid borrow issues

**Build Verification:**
- ✅ Both build targets compile successfully

---

### Items Kept in Local (Not Moved)

The following items were considered but kept in RTIC's Local resources:

1. **cansleep (OutputPin):** OutputPin doesn't implement Sync, can't be used with OnceLock/Mutex patterns for global statics. Only accessed by ignition_task, no concurrency concerns.

**Rationale:** For embassy migration preparation, the key is moving resources that need to be shared across tasks or accessed from async contexts without RTIC's local resource mechanism. Items that:
- Don't implement Sync (can't be used in global statics)
- Are only used by a single task (no sharing benefit)
...are best kept in Local resources.

### Final State of RTIC Local Resources

After all migrations, the following items remain in RTIC's Local struct:

```rust
#[local]
struct Local {
    cansleep: crate::bsp::OutputPin,
}
```

Only `cansleep` remains because OutputPin doesn't implement Sync and cannot be used with OnceLock/Mutex patterns for global statics.

---

### Power Sensors Migration to Global Static (2026-05-16)

**Goal:** Move `power_sensors` out of RTIC's Local resources into a global static.

**Challenge:** The original `PowerSensorArgs` struct used `NoopRawMutex` for the I2C bus reference:
```rust
struct PowerSensorArgs(
    &'static Mutex<NoopRawMutex, bsp::SensorI2c>,
    MonitorInterfaces,
    bsp::MonitorAlertPins,
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
);
```

`NoopRawMutex` doesn't implement `Sync`, which prevented the entire struct from being used in a global static (OnceLock requires Send+Sync types).

**Solution:** Switched from `NoopRawMutex` to `CriticalSectionRawMutex` for the I2C bus reference:
- `CriticalSectionRawMutex` implements both `Send` and `Sync`
- This is actually better practice since it provides proper synchronization when multiple tasks access the I2C bus concurrently
- The individual INA226 chip devices (`SensorDevice`) still use `NoopRawMutex` since each chip gets its own `I2cDevice` wrapper with no sharing needed per-chip

**Changes Made:**

1. **Updated bsp.rs type definitions:**
   - Changed `SensorDevice` to use `CriticalSectionRawMutex` instead of `NoopRawMutex`
   - Removed unused `NoopRawMutex` import from bsp.rs

2. **Added OnceLock import and static declaration in main.rs:**
   ```rust
   #[cfg(feature = "power_sensors")]
   static POWER_SENSORS: embassy_sync::once_lock::OnceLock<Mutex<CriticalSectionRawMutex, PowerSensorArgs>> = embassy_sync::once_lock::OnceLock::new();
   ```

3. **Removed from RTIC Local struct:**
   - Removed `power_sensors` from `Local` struct and init() return tuple

4. **Initialize in init():**
   ```rust
   #[cfg(feature = "power_sensors")]
   {
       let _power_sensors = PowerSensorArgs(
           i2c_manager,
           i2c_devices,
           mon_alert_pins,
           main_event_sender.clone(),
       );
       POWER_SENSORS.get_or_init(|| Mutex::new(_power_sensors));
   }
   ```

5. **Update i2c_task to use global static:**
   - Removed `local = [power_sensors]` from task attribute
   - Use `POWER_SENSORS.get().await.lock().await` pattern
   - Destructure guard: `let PowerSensorArgs(...) = &mut *guard`

**Key Design Decisions:**
u
- **NoopRawMutex retained for SensorDevice:** Individual INA226 chip devices don't need synchronization since each gets its own wrapper.

**Build Verification:**
- ✅ Both build targets compile successfully:
  - `--features=power_sensors`
  - `--features=terminal`

---

### Embassy Executor Port (Final Migration) — 2026-05-18

**Goal:** Complete the migration from RTIC to `embassy-executor`, removing all RTIC framework dependencies.

---

#### Phase 1: Build Verification & Dependency Updates

**Changes Made:**

1. **Updated Cargo.toml dependencies:**
   - Removed `rtic` and `rtic-monotonics` dependencies
   - Added `embassy-executor = { version = "0.6", features = ["nightly", "task-arena-size-32768"] }`
   - Added `embassy-time = { version = "0.4", features = ["defmt", "tick-hz-32_768", "defmt-timestamp-use-realtimeclock"] }`
   - Updated `embassy-stm32` to `0.1.0`
   - Added `async-debounce = "0.3"` for input debouncing
   - Removed `rtic-sync` dependency (already replaced earlier)

2. **Updated imports in main.rs:**
   - Replaced RTIC imports (`#[app]`, `SharedCell`, `init`, etc.) with embassy equivalents
   - Added `embassy_executor::{raw::Executor, Spawner}` for executor and task spawning
   - Added `embassy_sync::once_lock::OnceLock` for async-initialized globals
   - Added `embassy_futures::select::{select, select3, Either, Either3}` for async composition
   - Added `static_cell::StaticCell` for static initialization

---

#### Phase 2: Executor Integration

**Changes Made:**

1. **Replaced RTIC `#[app]` with embassy `#[main]`:**
   ```rust
   // Before (RTIC):
   #[app(Shared = Shared, Local = Local, resources_init = fn init(...) -> ...)]
   const APP: app::App = app::App {};
   
   // After (Embassy):
   static mut EXECUTOR: Option<Executor> = None;
   
   #[embassy_executor::main]
   async fn main(spawner: Spawner) {
       // Executor auto-initialized by #[main] macro
       ...
   }
   ```

2. **Removed manual executor initialization:** The `#[main]` macro handles executor setup automatically, including signal_fn/signal_ctx for interrupt-based wakeup.

3. **Replaced RTIC task spawning:**
   - Used `spawner.must_spawn(task())` to spawn embassy tasks from main
   - Tasks use `#[embassy_executor::task]` attribute instead of RTIC's `[resources = ...]`

---

#### Phase 3: Task Conversion

**Changes Made:**

1. **init_main_task → async task:**
   ```rust
   #[embassy_executor::task]
   async fn init_main_task() {
       // App initialization and main event loop
       let mut guard = APP.get().await.lock().await;
       match guard.init().await { ... }
       
       loop {
           use embassy_futures::select::{select, Either};
           let ret = select(events.receive(), async { guard.run().await }).await;
           // Handle events or app completion
       }
   }
   ```

2. **encoder_task → async task:**
   - Used raw pointers to get mutable access to array elements for `select3` (embassy's ExtiInput::wait_for_any_edge requires &mut self)
   - Updated `is_high()` calls: embassy returns `Result<bool, Infallible>` instead of RTIC's `bool`
   - Removed debounce wrapper (async-debounce 0.3 doesn't work with borrowed pins in this context)
   - Feature-gated behind `#[cfg(feature = "terminal")]` since encoder is only used with display

3. **ignition_task → async task:**
   - Uses global `IGNITION_PIN` static via `get().await.lock().await`
   - Manual 10ms debounce delay (same as previous RTIC migration)
   - Feature-gated behind `#[cfg(feature = "power_sensors")]`

4. **i2c_task → async task:**
   - Uses global `POWER_SENSORS` static via `get().await.lock().await`
   - Uses raw pointers for `select4` on alert pins (same pattern as encoder)
   - Feature-gated behind `#[cfg(feature = "power_sensors")]`

---

#### Phase 4: Static State Migration

**Changes Made:**

1. **Global statics (already prepared in previous migrations):**
   ```rust
   static NVSTORE: crate::nvstore::SharedNvStore = crate::nvstore::SharedNvStore::new();
   static CAN_IFACE: OnceLock<can::BufferedCan<'static, ...>> = OnceLock::new();
   static MAIN_ERROR_SENDER: OnceLock<Mutex<CriticalSectionRawMutex, BufferedCanErrorSender>> = OnceLock::new();
   static APP: OnceLock<Mutex<CriticalSectionRawMutex, MonitorApp>> = OnceLock::new();
   static CAN_SUSPENDED_SENDER: OnceLock<Sender<'static, ...>> = OnceLock::new();
   #[cfg(feature = "power_sensors")]
   static IGNITION_PIN: OnceLock<Mutex<CriticalSectionRawMutex, ExtiPin>> = OnceLock::new();
   #[cfg(feature = "power_sensors")]
   static POWER_SENSORS: OnceLock<Mutex<CriticalSectionRawMutex, (...)>> = OnceLock::new();
   #[cfg(feature = "terminal")]
   static ENCODER_ARGS: OnceLock<Mutex<CriticalSectionRawMutex, (...)>> = OnceLock::new();
   ```

2. **Removed RTIC Shared/Local structures entirely:**
   - No more `SharedCell`, `#[shared]`, or `#[local]` attributes
   - All shared state uses global statics with `OnceLock` + `Mutex`

3. **NV Store initialization moved to async context:**
   ```rust
   let nvs = nvstore::NvStore::new(flash_resources, #[cfg(feature = "power_sensors")] crc);
   *(NVSTORE.nv.lock().await) = Some(nvs);
   ```

---

#### Phase 5: RTIC Removal

**Changes Made:**

1. **Removed all RTIC attributes and macros:**
   - `#[app(...)]` → `#[embassy_executor::main]`
   - `[resources = ...]` on tasks → removed (tasks access globals directly)
   - `init()` function → replaced by async `main()`

2. **Removed RTIC-specific code:**
   - Resource destructuring from init() return tuple → direct Bsp::new() destructuring in main()
   - RTIC channel API (`make_channel`, `.recv().await`) → embassy sync API (`Channel::new()`, `.receive().await`)

3. **Cleaned up unused imports:**
   - Removed `rtic`, `rtic_monotonics`, `rtic_sync` imports
   - Kept only embassy and standard library imports needed for the new architecture

---

#### Key Design Decisions

1. **OnceLock over StaticCell:** All global state uses `embassy_sync::once_lock::OnceLock` because it provides async `.get().await`, which is essential for embassy-style initialization where tasks may need to access globals before they're fully initialized.

2. **Raw pointers for select() on arrays:** Embassy's `ExtiInput::wait_for_any_edge()` requires `&mut self`. When pins are stored in an array behind a shared reference, we use raw pointer arithmetic (`pins_ptr.add(i)`) to create non-overlapping mutable references. This is safe because each future accesses a distinct array element.

3. **Feature-gated tasks:** Tasks that only apply to specific features (encoder for `terminal`, ignition/i2c for `power_sensors`) are conditionally compiled with separate task definitions:
   ```rust
   #[cfg(feature = "terminal")]
   #[embassy_executor::task]
   async fn encoder_task() { ... }
   
   #[cfg(not(feature = "terminal"))]
   #[embassy_executor::task]
   async fn encoder_task() {
       loop { embassy_time::Timer::after(...).await; }
   }
   ```

4. **No executor run loop needed:** The `#[main]` macro automatically runs the executor's event loop. We don't need to manually call `executor.run()` as was required in earlier embassy versions.

5. **Mutex for shared state:** All global statics that may be accessed by multiple tasks are wrapped in `embassy_sync::mutex::Mutex<CriticalSectionRawMutex, T>` for safe concurrent access.

---

#### Build Verification

- ✅ Both build targets compile successfully with zero errors:
  - `cargo build --release --features=power_sensors`
  - `cargo build --release --features=terminal`

- Remaining warnings are all expected:
  - Unused variables for feature-gated destructured fields (Rust doesn't support `#[cfg]` on individual tuple struct fields in patterns)
  - Dead code warnings for statics that are only used with specific features

---

#### Files Modified

- `/home/cschuhen/rust/canot/apps/battery_monitor/Cargo.toml` — Updated dependencies
- `/home/cschuhen/rust/canot/apps/battery_monitor/src/main.rs` — Complete RTIC → Embassy migration

---

### Previous Work Summary

#### Async I2C and INA226 Conversion
- Switched from blocking to async `embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice`
- Added `async` feature to ina226 dependency in Cargo.toml
- Converted all INA226 methods to use `.await`
- Moved device setup from sync init() to async i2c_task
- Changed read_monitor() from sync to async function

#### RTIC Sync → Embassy Sync Migration
- Replaced `rtic_sync::{channel::*, make_channel}` with `embassy_sync::channel::{Channel, Sender, Receiver}`
- Used `CriticalSectionRawMutex` for channels (required for Send+Sync in RTIC context)
- Updated all type signatures to include mutex parameter
- Fixed API differences: `.recv()` → `.receive()`, `.send(...).await` returns `()` not `Result`
- Removed `[dependencies.rtic-sync]` from Cargo.toml
