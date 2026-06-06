//! Storage initialization — sets up NVS, loads last monitor observation.
//!
//! This module contains the core logic for initializing flash storage, extracted from
//! `main.rs` to keep the RTIC app definition lean.

use crate::error::{mkerr, ErrorCode};
use crate::nvstore::SharedNvStore;
#[cfg(feature = "power_sensors")]
use crate::APPDATA;
use embassy_stm32::gpio::Output;
use j1939_async as j1939;

const FILE_CODE: u8 = 0x01;

/// Initialize storage and load the last monitor observation.
///
/// Flashes LEDs through indices 0-3 to indicate progress, then returns
/// the stored time counter (if any) or `None`.
pub async fn run_init_storage(
    shared_nvs: &SharedNvStore,
    leds: &mut [Output<'static>; 5],
) -> Result<Option<i64>, j1939::error::Error> {
    leds[0].set_low();
    let mut unlocked = shared_nvs.nv.lock().await;
    let nvs = unlocked
        .as_mut()
        .ok_or(mkerr(FILE_CODE, ErrorCode::NoDevice, line!()))?;

    nvs.init().await?;

    leds[1].set_low();

    //nvs.load_monitor_settings().await?;

    leds[2].set_high();

    #[cfg(feature = "power_sensors")]
    let ret = {
        let mut header = crate::nvstore::power_sensors::Header::new();
        match nvs
            .load_last_monitor_observation(&APPDATA, -1, &mut header)
            .await?
        {
            true => Some(header.time),
            false => None,
        }
    };
    #[cfg(not(feature = "power_sensors"))]
    let ret: Option<i64> = None;

    leds[3].set_high();

    Ok(ret)
}
