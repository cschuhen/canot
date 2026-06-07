//! Power sensor monitoring for INA226 devices.
//!
//! Contains the alert-driven read loop and per-monitor reading logic,
//! all gated behind the `power_sensors` feature flag.

use crate::application::MainEvent;
use crate::PowerSensorArgs;
use embassy_futures::select::{select4, Either4};

/// Read a single INA226 monitor and send the observation via the event sender.
/// Called when an alert pin goes low. Must read mask_enable to clear the alert flag.
pub async fn read_monitor(
    chip: &mut crate::MonitorChip,
    index: u8,
    event_sender: &mut crate::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        MainEvent,
        { crate::MAIN_EVENT_CAPACITY },
    >,
) {
    // Must read this to clear alert.
    let mon_mask = chip.mask_enable().await.unwrap();

    if !mon_mask.contains(ina226::MaskEnableFlags::CVRF) {
        return;
    }

    let bus = chip.bus_voltage_raw().await;
    let shunt = chip.shunt_voltage_raw().await;
    match (bus, shunt) {
        (Ok(bus), Ok(shunt)) => match event_sender.try_send(MainEvent::MonitorObservation(
            crate::powercalc::Observation::new(index, bus, shunt),
        )) {
            Ok(_) => {}
            Err(_) => {
                //defmt::println!("Spawn Err ")
            }
        },
        (Err(_bus), Err(_shunt)) => {
            defmt::println!("FAIL bus: shunt:")
        }
        (Ok(_), Err(_shunt)) => {
            defmt::println!("FAIL shunt:");
        }
        (Err(_bus), Ok(_)) => {
            defmt::println!("FAIL bus:");
        }
    }
}

/// Main power sensor loop: wait for any of 4 alert pins to go low, then read the corresponding monitor.
pub async fn run_power_sensors(args: &mut crate::PowerSensorArgs) {
    let PowerSensorArgs(_, devices, alert_pins, event_sender) = args;
    let [m0, m1, m2, m3] = alert_pins;

    loop {
        let index = match select4(
            m0.wait_for_low(),
            m1.wait_for_low(),
            m2.wait_for_low(),
            m3.wait_for_low(),
        )
        .await
        {
            Either4::First(_) => 0,
            Either4::Second(_) => 1,
            Either4::Third(_) => 2,
            Either4::Fourth(_) => 3,
        };
        if index >= devices.len() {
            defmt::println!("NoDvc {}/{}", index, devices.len());
            continue;
        }
        read_monitor(&mut devices[index].chip, index as u8, event_sender).await;
    }
}
