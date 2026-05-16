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
