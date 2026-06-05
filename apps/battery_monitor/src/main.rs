//! Battery Monitor ECU (Embassy Executor Version)
//!
//! Monitor multiple voltages and current via connected INA226 modules.
//!

#![no_std]
#![cfg_attr(not(doc), no_main)]
#![feature(type_alias_impl_trait)]

use {defmt_rtt as _, panic_probe as _};

pub mod application;
mod bsp;
pub mod consts;
pub mod error;
pub mod graphical_elements;
pub mod nvstore;
pub mod powercalc;
pub mod types;
#[cfg(feature = "terminal")]
pub mod ui;

use crate::application::MainEvent;
use crate::error::*;

// Embassy imports
#[allow(unused_imports)]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{raw::Executor, Spawner};
use embassy_stm32::can;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[cfg(any(feature = "power_sensors", feature = "terminal"))]
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
#[cfg(feature = "power_sensors")]
use ina226::INA226;
use static_cell::StaticCell;

const FILE_CODE: u8 = 0x01;
pub const MAIN_EVENT_CAPACITY: usize = 9;
pub const CAN_TX_BUF_SIZE: usize = 8;
pub const CAN_RX_BUF_SIZE: usize = 20;

type Error = crate::error::Error;
#[cfg(feature = "power_sensors")]
type MonitorChip = INA226<bsp::SensorDevice>;

#[cfg(feature = "power_sensors")]
pub struct MonitorInterface {
    chip: MonitorChip,
}

#[cfg(feature = "power_sensors")]
type MonitorInterfaces = heapless::Vec<MonitorInterface, { consts::MAX_MONITORS }>;

pub enum I2cEvent {
    Alert(u8),
    Timeout,
}

pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

// Global statics for shared state (moved from RTIC)
static NVSTORE: crate::nvstore::SharedNvStore = crate::nvstore::SharedNvStore::new();

// Executor - using static mut with unsafe initialization
static mut EXECUTOR: Option<Executor> = None;

static APP: OnceLock<Mutex<CriticalSectionRawMutex, crate::application::MonitorApp>> =
    OnceLock::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // FIXME: Executor initialization requires signal_fn and signal_ctx in embassy-executor 0.1.x
    // The #[main] macro handles this automatically, so we skip manual init.
    #[allow(static_mut_refs, unused_unsafe)]
    let _executor_ref = &raw const EXECUTOR;

    #[allow(unused_variables, unused_mut)]
    let bsp::Bsp(
        device_id,
        mut can_iface,
        cansleep,
        // FIXME: Rust doesn't support #[cfg] on individual tuple struct fields in patterns.
        // Destructure all fields unconditionally; ignition_pin is only used with power_sensors feature.
        _ignition_pin,
        _leds,
        _nvstore_i2c,
        mut sensors_i2c,
        mon_alert_pins,
        encoder_pins,
        flash_resources,
        crc,
        display_connector,
    ) = bsp::Bsp::new();

    #[cfg(feature = "power_sensors")]
    {
        static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, bsp::SensorI2c>> =
            StaticCell::new();
        let i2c_manager = {
            for addr in bsp::consts::INA226_ADDRS {
                let mut dummy = [0u8; 0];
                let dummy2 = [0u8; 0];
                if sensors_i2c
                    .blocking_write_read(addr, &dummy2, &mut dummy)
                    .is_ok()
                {
                    // Device found
                }
            }
            I2C_BUS.init(Mutex::new(sensors_i2c))
        };

        IGNITION_PIN.get_or_init(|| Mutex::new(_ignition_pin));

        let i2c_devices = MonitorInterfaces::new();
        POWER_SENSORS.get_or_init(|| Mutex::new((i2c_manager, i2c_devices, mon_alert_pins)));
    }

    // Setup display for terminal feature (no block to keep variables in scope)
    #[cfg(feature = "terminal")]
    let st = embassy_time::Delay;
    #[cfg(feature = "terminal")]
    let spi = embedded_hal_bus::spi::ExclusiveDevice::new(
        display_connector.spi,
        display_connector.cs,
        st.clone(),
    );
    #[cfg(feature = "terminal")]
    let interface = display_interface_spi::SPIInterface::new(spi, display_connector.dc);
    #[cfg(feature = "terminal")]
    let raw_disp = oled_async::Builder::new(crate::application::SpecifficDisplay {})
        .with_rotation(crate::application::DISPLAY_ROTATION)
        .connect(interface);
    #[cfg(feature = "terminal")]
    let display: oled_async::mode::graphics::GraphicsMode<
        _,
        _,
        { crate::application::DISPLAY_BUFFFER_SIZE },
    > = raw_disp.into();

    // Initialize CAN interface
    can_iface.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    can_iface.set_bitrate(250_000);

    let can_iface = can_iface.start(can::OperatingMode::NormalOperationMode);
    static TX_BUF: StaticCell<can::TxBuf<CAN_TX_BUF_SIZE>> = StaticCell::new();
    static RX_BUF: StaticCell<can::RxBuf<CAN_RX_BUF_SIZE>> = StaticCell::new();
    let can_iface = can_iface.buffered(
        TX_BUF.init(can::TxBuf::<CAN_TX_BUF_SIZE>::new()),
        RX_BUF.init(can::RxBuf::<CAN_RX_BUF_SIZE>::new()),
    );

    let can_reader = can_iface.reader();
    let can_writer = can_iface.writer();
    #[allow(unused_mut)]
    let mut error_sender = crate::bsp::BufferedCanErrorSender::new(can_writer.clone());

    CAN_IFACE.get_or_init(|| can_iface);

    MAIN_ERROR_SENDER.get_or_init(|| Mutex::new(error_sender.clone()));

    // Setup event channel
    static MAIN_CHANNEL: StaticCell<
        embassy_sync::channel::Channel<CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
    > = StaticCell::new();
    let main_channel = MAIN_CHANNEL.init(embassy_sync::channel::Channel::new());
    let main_event_sender = main_channel.sender();

    CAN_SUSPENDED_SENDER.get_or_init(|| main_event_sender.clone());

    // Initialize NV Store in async context
    let nvs = nvstore::NvStore::new(
        flash_resources,
        #[cfg(feature = "power_sensors")]
        crc,
    );

    *(NVSTORE.nv.lock().await) = Some(nvs);

    defmt::println!("CHK {}", line!());
    // Initialize app
    let app = crate::application::MonitorApp::new(
        embassy_time::Instant::now(),
        can_reader,
        can_writer,
        device_id,
        #[cfg(feature = "terminal")]
        display,
        #[cfg(feature = "terminal")]
        display_connector.rst,
        &APPDATA,
    );
    defmt::println!("CHK {}", line!());

    defmt::println!("CHK {}", line!());
    APP.get_or_init(|| Mutex::new(app));
    defmt::println!("CHK {}", line!());

    // Spawn tasks using Spawner.must_spawn() with #[task] generated tokens
    spawner.must_spawn(init_main_task());

    #[cfg(feature = "terminal")]
    {
        let _ = ENCODER_ARGS.get_or_init(|| Mutex::new((encoder_pins, main_event_sender.clone())));
    }

    defmt::println!("CHK {}", line!());
    #[cfg(feature = "terminal")]
    spawner.must_spawn(encoder_task());

    defmt::println!("CHK {}", line!());
    #[cfg(feature = "power_sensors")]
    spawner.must_spawn(ignition_task(cansleep));

    defmt::println!("CHK {}", line!());
    #[cfg(feature = "power_sensors")]
    spawner.must_spawn(i2c_task());

    defmt::println!("CHK {}", line!());
    // FIXME: Executor::run() doesn't exist in embassy-executor 0.1.x raw API.
    // The #[main] macro already runs the executor loop, so we just spin here.
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
        defmt::println!("CHK {}", line!());
    }
    /*loop {
        core::hint::spin_loop();
        defmt::println!("CHK {}", line!());
    }*/
}

#[embassy_executor::task]
async fn init_main_task() {
    defmt::println!("CHK {} Init Main", line!());
    defmt::println!("CHK {}", line!());
    // Initialize app
    let mut guard = APP.get().await.lock().await;
    match guard.init().await {
        Err(err) => {
            let mut error_guard = MAIN_ERROR_SENDER.get().await.lock().await;
            error_guard.send(&err);
        }
        _ => {}
    }
    defmt::println!("CHK {}", line!());

    #[cfg(feature = "power_sensors")]
    {
        let mut unlocked = APPDATA.lock().await;
        match unlocked
            .monitors
            .setup(0, embassy_time::Instant::now())
            .await
        {
            Err(e) => {
                let mut guard = MAIN_ERROR_SENDER.get().await.lock().await;
                guard.send(&e);
            }
            _ => {}
        }
    }

    // Initialize storage
    let _ = init_storage(&NVSTORE).await;

    defmt::println!("CHK {}", line!());
    // Setup event channel for init_main_task
    static INIT_EVENT_CHANNEL: StaticCell<
        embassy_sync::channel::Channel<CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
    > = StaticCell::new();
    let main_channel = INIT_EVENT_CHANNEL.init(embassy_sync::channel::Channel::new());
    let events = main_channel.receiver();

    use embassy_futures::select::{select, Either};

    loop {
        defmt::println!("CHK {}", line!());

        let ret = select(events.receive(), async {
            let mut guard = APP.get().await.lock().await;
            if let Err(e) = guard.run().await {
                let mut error_guard = MAIN_ERROR_SENDER.get().await.lock().await;
                error_guard.send(&e);
            }
        })
        .await;
        defmt::println!("CHK {}", line!());

        match ret {
            Either::First(event) => {
                let mut guard = APP.get().await.lock().await;
                guard.on_event(&event).await;
            }
            Either::Second(()) => {}
        }
    }
}

async fn init_storage(shared_nvs: &nvstore::SharedNvStore) -> Result<(), crate::error::Error> {
    let mut unlocked = shared_nvs.nv.lock().await;
    let nvs = unlocked.as_mut().ok_or(crate::error::mkerr(
        FILE_CODE,
        crate::ErrorCode::NoDevice,
        line!(),
    ))?;
    nvs.init().await?;

    #[cfg(feature = "power_sensors")]
    {
        let _ret = nvs
            .load_last_monitor_observation(&APPDATA, -1, &mut nvstore::power_sensors::Header::new())
            .await;
    }
    Ok(())
}

#[cfg(feature = "terminal")]
#[embassy_executor::task]
async fn encoder_task() {
    let mut guard = ENCODER_ARGS.get().await.lock().await;
    let (pins, sender) = &mut *guard;

    run_encoder(pins, sender).await;
}

#[cfg(not(feature = "terminal"))]
#[embassy_executor::task]
async fn encoder_task() {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

#[cfg(feature = "terminal")]
async fn run_encoder(
    pins: &crate::bsp::InputPins,
    sender: &Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
) {
    // Use raw pointers to get mutable access to each pin independently
    // This is safe because each pin is accessed in a separate future
    let pins_ptr = pins.as_ptr() as *mut embassy_stm32::exti::ExtiInput<'static>;
    let enc_a = unsafe { &mut *(pins_ptr.add(1)) };
    let enc_b = unsafe { &mut *(pins_ptr.add(2)) };
    let button = unsafe { &mut *(pins_ptr) };

    #[derive(PartialEq)]
    enum EncoderState {
        A,
        B,
        Idle,
    }

    let mut press_time = Option::<embassy_time::Instant>::None;
    let mut encoder_state: EncoderState = EncoderState::Idle;

    loop {
        use crate::application::HumanEvent;
        use embassy_futures::select::{select3, Either3};
        use embedded_hal::digital::InputPin;

        // Wait for any edge on encoder pins
        match select3(
            enc_a.wait_for_any_edge(),
            enc_b.wait_for_any_edge(),
            button.wait_for_any_edge(),
        )
        .await
        {
            Either3::First(_) => {
                if enc_a.is_high().unwrap_or(false) {
                    if encoder_state == EncoderState::B {
                        encoder_state = EncoderState::Idle;
                        let _ = sender
                            .send(MainEvent::HID(HumanEvent::EncoderAntiClockwise))
                            .await;
                    } else {
                        encoder_state = EncoderState::A;
                    }
                } else {
                    encoder_state = EncoderState::Idle;
                }
            }
            Either3::Second(_) => {
                if enc_b.is_high().unwrap_or(false) {
                    if encoder_state == EncoderState::A {
                        encoder_state = EncoderState::Idle;
                        let _ = sender
                            .send(MainEvent::HID(HumanEvent::EncoderClockwise))
                            .await;
                    } else {
                        encoder_state = EncoderState::B;
                    }
                } else {
                    encoder_state = EncoderState::Idle;
                }
            }
            Either3::Third(_) => {
                if button.is_high().unwrap_or(false) {
                    press_time = Some(embassy_time::Instant::now());
                    let _ = sender
                        .send(MainEvent::HID(HumanEvent::EncoderButtonPressed))
                        .await;
                } else {
                    if let Some(start) = press_time.take() {
                        let duration = embassy_time::Instant::now() - start;
                        let _ = sender
                            .send(MainEvent::HID(HumanEvent::EncoderButtonReleased(duration)))
                            .await;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn ignition_task(mut cansleep: embassy_stm32::gpio::Output<'static>) {
    #[cfg(feature = "power_sensors")]
    {
        let mut enabled = true;
        defmt::println!("Ignition task started");

        loop {
            // Wait for any edge on the ignition pin
            let ignition_pin_ref = IGNITION_PIN.get().await;
            match ignition_pin_ref.lock().await.wait_for_any_edge().await {
                () => {
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await; // Debounce

                    let ignition = ignition_pin_ref.lock().await.is_high();
                    if ignition != enabled {
                        defmt::println!("Ignition state changed: {}", ignition);
                        enabled = ignition;

                        if enabled {
                            cansleep.set_low();
                            embassy_time::Timer::after(embassy_time::Duration::from_millis(100))
                                .await;
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
    #[cfg(not(feature = "power_sensors"))]
    {
        let _cansleep = &mut cansleep;
        loop {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
        }
    }
}

#[embassy_executor::task]
async fn i2c_task() {
    #[cfg(feature = "power_sensors")]
    {
        let mut guard = POWER_SENSORS.get().await.lock().await;
        let (_i2c_manager, devices, alert_pins) = &mut *guard;

        // Verify and configure each device
        let mut to_remove: heapless::Vec<usize, { consts::MAX_MONITORS }> = heapless::Vec::new();
        for idx in 0..devices.len() {
            match devices[idx].chip.die_id().await {
                Ok(_id) => {
                    defmt::println!("Found dvc {:x}", _id);

                    let (bus, shunt) = (
                        devices[idx].chip.bus_voltage_raw().await,
                        devices[idx].chip.shunt_voltage_raw().await,
                    );

                    if !matches!((bus, shunt), (Ok(_), Ok(_))) {
                        defmt::println!("I2CRead error");
                        let _ = to_remove.push(idx);
                        continue;
                    }

                    // Configure device
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

        // Remove invalid devices
        let mut count = to_remove.len();
        while count > 0 {
            count -= 1;
            devices.remove(to_remove[count]);
        }

        defmt::println!("Power sensors: {} devices found", devices.len());

        run_power_sensors(devices, alert_pins).await;
    }
}

#[cfg(feature = "power_sensors")]
async fn run_power_sensors(devices: &mut MonitorInterfaces, alert_pins: &bsp::MonitorAlertPins) {
    // FIXME: embassy ExtiInput::wait_for_low() requires &mut self, but we only have &mut through Mutex guard.
    // Using unsafe raw pointer to create non-overlapping mutable borrows of array elements.
    // Each element is accessed independently in separate futures, so this is safe.
    let pins_ptr = alert_pins.as_ptr() as *const embassy_stm32::exti::ExtiInput<'static>;

    loop {
        use embassy_futures::select::{select4, Either4};

        // Create futures from raw pointers - each points to a distinct array element
        let f0 = unsafe {
            (&mut *(pins_ptr as *mut embassy_stm32::exti::ExtiInput<'static>)).wait_for_low()
        };
        let f1 = unsafe {
            (&mut *((pins_ptr as *mut embassy_stm32::exti::ExtiInput<'static>).add(1)))
                .wait_for_low()
        };
        let f2 = unsafe {
            (&mut *((pins_ptr as *mut embassy_stm32::exti::ExtiInput<'static>).add(2)))
                .wait_for_low()
        };
        let f3 = unsafe {
            (&mut *((pins_ptr as *mut embassy_stm32::exti::ExtiInput<'static>).add(3)))
                .wait_for_low()
        };

        let index = match select4(f0, f1, f2, f3).await {
            Either4::First(_) => 0,
            Either4::Second(_) => 1,
            Either4::Third(_) => 2,
            Either4::Fourth(_) => 3,
        };

        if index >= devices.len() {
            defmt::println!("NoDvc {}/{}", index, devices.len());
            continue;
        }

        read_monitor(&mut devices[index].chip, index as u8).await;
    }
}

#[cfg(feature = "power_sensors")]
async fn read_monitor(chip: &mut MonitorChip, index: u8) {
    let mon_mask = chip.mask_enable().await.unwrap();
    if !mon_mask.contains(ina226::MaskEnableFlags::CVRF) {
        return;
    }

    let bus = chip.bus_voltage_raw().await;
    let shunt = chip.shunt_voltage_raw().await;

    match (bus, shunt) {
        (Ok(bus), Ok(shunt)) => {
            let obs = crate::powercalc::Observation::new(index, bus, shunt);
            CAN_SUSPENDED_SENDER
                .get()
                .await
                .send(MainEvent::MonitorObservation(obs))
                .await;
        }
        _ => defmt::println!("FAIL"),
    }
}

#[allow(dead_code)]
static CAN_IFACE: OnceLock<can::BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>> =
    OnceLock::new();
static MAIN_ERROR_SENDER: OnceLock<
    Mutex<CriticalSectionRawMutex, crate::bsp::BufferedCanErrorSender>,
> = OnceLock::new();

#[cfg(feature = "power_sensors")]
static IGNITION_PIN: OnceLock<Mutex<CriticalSectionRawMutex, bsp::ExtiPin>> = OnceLock::new();
#[allow(dead_code)]
static CAN_SUSPENDED_SENDER: OnceLock<
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
> = OnceLock::new();

#[cfg(feature = "power_sensors")]
static POWER_SENSORS: OnceLock<
    Mutex<
        CriticalSectionRawMutex,
        (
            &'static Mutex<CriticalSectionRawMutex, bsp::SensorI2c>,
            MonitorInterfaces,
            bsp::MonitorAlertPins,
        ),
    >,
> = OnceLock::new();

#[cfg(feature = "terminal")]
static ENCODER_ARGS: OnceLock<
    Mutex<
        CriticalSectionRawMutex,
        (
            crate::bsp::InputPins,
            Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
        ),
    >,
> = OnceLock::new();

static APPDATA: application::DataStore =
    embassy_sync::mutex::Mutex::new(application::Data::new(&NVSTORE));

#[allow(dead_code)]
static IGNITION_STATE: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(true);
