//! Ignition input handling — monitors ignition pin, toggles CAN sleep/wake.
//!
//! This module contains the core logic for the ignition task, extracted from
//! `main.rs` to keep the RTIC app definition lean.

use crate::application::MainEvent;
use crate::bsp;

use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::Delay as TimeDelay;
use embedded_hal_async::delay::DelayNs;

/// Global static for ignition state.
pub static IGNITION_STATE: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(true);

/// Global static for ignition pin (OnceLock + Mutex for safe shared access).
#[cfg(feature = "power_sensors")]
pub static IGNITION_PIN: OnceLock<Mutex<CriticalSectionRawMutex, bsp::ExtiPin>> = OnceLock::new();

/// Run the ignition monitoring loop.
///
/// Waits for edges on the global ignition pin, debounces, and then:
/// - Toggles `cansleep` to put CAN into/out of sleep mode.
/// - Updates the shared `IGNITION_STATE`.
/// - Sends a `MainEvent::CanEnabled` through the global suspended sender.
pub async fn run_ignition(cansleep: &mut Output<'static>) {
    {
        use crate::CAN_SUSPENDED_SENDER;

        let mut enabled = true;
        defmt::println!("Ignition task started");
        let mut delay = TimeDelay {};

        loop {
            // Wait for any edge on the ignition pin (manual debounce)
            let ignition_pin_ref = IGNITION_PIN.get().await;
            match ignition_pin_ref.lock().await.wait_for_any_edge().await {
                () => {
                    // Debounce: wait a short time and re-check
                    delay.delay_ms(10).await;
                    let ignition = ignition_pin_ref.lock().await.is_high();

                    if ignition != enabled {
                        defmt::println!("Ignition state changed: {}", ignition);
                        enabled = ignition;

                        if enabled {
                            cansleep.set_low();
                            delay.delay_ms(100).await;
                        } else {
                            cansleep.set_high();
                        }

                        *IGNITION_STATE.lock().await = enabled;

                        CAN_SUSPENDED_SENDER
                            .get()
                            .await
                            .send(MainEvent::CanEnabled(enabled))
                            .await;
                    }
                }
            }
        }
    }
}
