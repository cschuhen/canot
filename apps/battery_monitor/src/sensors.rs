//! Power sensor monitoring for INA226 devices.
//!
//! Contains device initialization from BSP resources, the alert-driven read loop,
//! per-monitor reading logic, all gated behind the `power_sensors` feature flag.

use crate::application::MainEvent;
use crate::bsp;
use crate::consts::MAX_MONITORS;
use crate::PowerSensorArgs;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

/// Type alias for the INA226 monitor chip.
pub type MonitorChip = ina226::INA226<bsp::SensorDevice>;

/// Wrapper struct holding a single INA226 monitor interface.
pub struct MonitorInterface {
    pub chip: MonitorChip,
}

/// Collection of all monitor interfaces.
pub type MonitorInterfaces = heapless::Vec<MonitorInterface, { MAX_MONITORS }>;
use embassy_futures::select::{select4, Either4};
use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use heapless::Vec;
use static_cell::StaticCell;

/// Build PowerSensorArgs from BSP resources.
///
/// Scans for INA226 devices on the I2C bus, creates wrapper structs,
/// and returns a fully constructed `PowerSensorArgs` ready for init_power_sensors + run_power_sensors.
pub fn init_power_sensor_args(
    mut sensors_i2c: crate::bsp::SensorI2c,
    mon_alert_pins: [ExtiInput<'static, embassy_stm32::mode::Async>; 4],
    main_event_sender: Sender<
        'static,
        CriticalSectionRawMutex,
        MainEvent,
        { crate::MAIN_EVENT_CAPACITY },
    >,
) -> PowerSensorArgs {
    type Addresses = Vec<u8, { MAX_MONITORS }>;

    // Scan for I2C addresses using blocking methods (works on async I2c too).
    let mut addresses = Addresses::new();
    for addr in crate::bsp::consts::INA226_ADDRS {
        let mut dummy = [0u8; 0];
        let dummy2 = [0u8; 0];
        match sensors_i2c.blocking_write_read(addr, &dummy2, &mut dummy) {
            Ok(()) => {
                addresses.push(addr).unwrap();
            }
            Err(_e) => {}
        }
    }

    // Setup I2C Bus manager (async I2c, blocking methods work during init)
    static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, crate::bsp::SensorI2c>> =
        StaticCell::new();
    let i2c_manager = I2C_BUS.init(Mutex::new(sensors_i2c));

    // Setup I2C devices (create wrappers only, no I2C traffic yet - RTIC init is sync)
    let mut i2c_devices = Vec::new();
    for addr in addresses {
        if i2c_devices
            .push(MonitorInterface {
                chip: ina226::INA226::new(I2cDevice::new(i2c_manager), addr),
            })
            .is_err()
        {
            defmt::println!("Max monitors reached");
        }
    }

    PowerSensorArgs(i2c_manager, i2c_devices, mon_alert_pins, main_event_sender)
}

/// Read a single INA226 monitor and send the observation via the event sender.
/// Called when an alert pin goes low. Must read mask_enable to clear the alert flag.
pub async fn read_monitor(
    chip: &mut MonitorChip,
    index: u8,
    event_sender: &mut Sender<
        'static,
        CriticalSectionRawMutex,
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

/// Discover and configure INA226 devices on startup. Runs once before entering the monitoring loop.
pub async fn init_power_sensors(args: &mut PowerSensorArgs) {
    let PowerSensorArgs(_i2c_manager, devices, _alert_pins, _event_sender) = args;

    // Verify each device and configure it (runs once at startup)
    // Collect indices to remove first to avoid borrow issues
    let mut to_remove: Vec<usize, { MAX_MONITORS }> = Vec::new();
    for idx in 0..devices.len() {
        match devices[idx].chip.die_id().await {
            Ok(_id) => {
                defmt::println!("Found dvc {:x}", _id);
                // Do an initial read to check comms
                let (bus, shunt) = (
                    devices[idx].chip.bus_voltage_raw().await,
                    devices[idx].chip.shunt_voltage_raw().await,
                );
                match (bus, shunt) {
                    (Ok(_), Ok(_)) => {}
                    _ => {
                        defmt::println!("I2CRead error");
                        let _ = to_remove.push(idx);
                        continue;
                    }
                }

                // Configure conversions and alert.
                let mut mon_cfg = devices[idx].chip.configuration().await.unwrap().unwrap();
                mon_cfg.mode = ina226::MODE::ShuntBusVoltageContinuous;
                mon_cfg.avg = ina226::AVG::_1024;
                mon_cfg.vbusct = ina226::VBUSCT::_140us;
                mon_cfg.vshct = ina226::VSHCT::_588us;
                devices[idx].chip.set_configuration(&mon_cfg).await.unwrap();

                devices[idx]
                    .chip
                    .set_mask_enable(ina226::MaskEnableFlags::CNVR)
                    .await
                    .unwrap();
            }
            Err(_e) => {
                defmt::println!("No dvc");
                let _ = to_remove.push(idx);
            }
        }
    }

    // Remove invalid devices (in reverse order to preserve indices)
    let mut count = to_remove.len();
    while count > 0 {
        count -= 1;
        devices.remove(to_remove[count]);
    }

    defmt::println!("Power sensors: {} devices found", devices.len());
}

/// Main power sensor loop: wait for any of 4 alert pins to go low, then read the corresponding monitor.
pub async fn run_power_sensors(args: &mut PowerSensorArgs) {
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
