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

1. **main_error_sender (BufferedCanErrorSender):** Only used by main_task, no sharing needed across tasks
2. **cansleep (OutputPin):** OutputPin doesn't implement Sync, can't be used with OnceLock/Mutex patterns
3. **app (MonitorApp):** Only used by main_task, no sharing needed
4. **power_sensors (PowerSensorArgs):** Feature-gated, complex struct with multiple fields
5. **encoder_args (EncoderArgs):** Feature-gated for terminal feature only

**Rationale:** For embassy migration preparation, the key is moving resources that need to be shared across tasks or accessed from async contexts without RTIC's local resource mechanism. Single-task resources don't benefit as much from being moved to global statics.

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
