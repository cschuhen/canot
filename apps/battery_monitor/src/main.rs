//! Battery Monitor ECU
//!
//! Monitor multiple voltages and current via conected INA226 modules.
//!

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![cfg_attr(not(doc), no_main)]
//#![feature(type_alias_impl_trait)]

use {defmt_rtt as _, panic_probe as _};
// panic_probe seems to add 7k of flash usage.
//use panic_reset as _;
//use panic_probe as _;

use crate::application::MainEvent;
use crate::error::*;
use rtic::app;

pub mod application;
mod bsp;
pub mod consts;
pub mod error;
pub mod graphical_elements;
pub mod nvstore;
pub mod powercalc;
#[cfg(feature = "power_sensors")]
pub mod sensors;
pub mod types;
#[cfg(feature = "terminal")]
pub mod ui;

#[cfg(feature = "power_sensors")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::can;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[cfg(any(feature = "power_sensors", feature = "terminal"))]
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

#[cfg(feature = "power_sensors")]
use embassy_sync::once_lock::OnceLock;

#[cfg(feature = "power_sensors")]
use ina226::INA226;
#[cfg(feature = "power_sensors")]
pub type MonitorChip = INA226<bsp::SensorDevice>;

const FILE_CODE: u8 = 0x01;

pub const MAIN_EVENT_CAPACITY: usize = 9;
pub const CAN_TX_BUF_SIZE: usize = 8;
pub const CAN_RX_BUF_SIZE: usize = 20;

type Error = crate::error::Error;

#[cfg(feature = "power_sensors")]
pub struct MonitorInterface {
    chip: MonitorChip,
}

#[cfg(feature = "power_sensors")]
pub type MonitorInterfaces = heapless::Vec<MonitorInterface, { consts::MAX_MONITORS }>;

pub enum I2cEvent {
    Alert(u8),
    Timeout,
}

pub mod pac {
    // pub use cortex_m_rt::interrupt;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

struct MainArgs {
    leds: crate::bsp::Leds,
    #[cfg(feature = "power_sensors")]
    ndevices: usize,
    nvs: nvstore::NvStore,
}

#[cfg(feature = "terminal")]
struct EncoderArgs(
    crate::bsp::InputPins,
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
    crate::bsp::BufferedCanErrorSender,
);

#[cfg(feature = "power_sensors")]
pub struct PowerSensorArgs(
    &'static Mutex<CriticalSectionRawMutex, bsp::SensorI2c>,
    MonitorInterfaces,
    bsp::MonitorAlertPins,
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
);

static NVSTORE: crate::nvstore::SharedNvStore = crate::nvstore::SharedNvStore::new();

// Global static for data store (moved out of RTIC Shared resources)
static APPDATA: application::DataStore =
    embassy_sync::mutex::Mutex::new(application::Data::new(&NVSTORE));

// Global static for ignition state (moved out of RTIC Shared resources)
static IGNITION_STATE: Mutex<CriticalSectionRawMutex, bool> = Mutex::new(true);

// Global static for ignition pin (moved out of RTIC Local resources)
// Using OnceLock with Mutex for safe shared access in preparation for embassy migration
#[cfg(feature = "power_sensors")]
static IGNITION_PIN: OnceLock<Mutex<CriticalSectionRawMutex, bsp::ExtiPin>> = OnceLock::new();

// Global static for can suspended event sender (moved out of RTIC Local resources)
// Using OnceLock for async access pattern consistency with other globals
static CAN_SUSPENDED_SENDER: embassy_sync::once_lock::OnceLock<
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
> = embassy_sync::once_lock::OnceLock::new();

// Global static for CAN interface (moved out of RTIC Local resources)
// Using OnceLock since BufferedCan is Clone+Send and used by multiple tasks
static CAN_IFACE: embassy_sync::once_lock::OnceLock<
    can::BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>,
> = embassy_sync::once_lock::OnceLock::new();

// Global static for main error sender (moved out of RTIC Local resources)
// Using OnceLock with Mutex since BufferedCanErrorSender needs interior mutability
static MAIN_ERROR_SENDER: embassy_sync::once_lock::OnceLock<
    Mutex<CriticalSectionRawMutex, crate::bsp::BufferedCanErrorSender>,
> = embassy_sync::once_lock::OnceLock::new();

// Global static for application (moved out of RTIC Local resources)
// Using OnceLock with Mutex since MonitorApp needs interior mutability
static APP: embassy_sync::once_lock::OnceLock<
    Mutex<CriticalSectionRawMutex, crate::application::MonitorApp>,
> = embassy_sync::once_lock::OnceLock::new();

// Global static for encoder args (moved out of RTIC Local resources)
// Using OnceLock with Mutex since EncoderArgs needs interior mutability
#[cfg(feature = "terminal")]
static ENCODER_ARGS: embassy_sync::once_lock::OnceLock<
    Mutex<CriticalSectionRawMutex, EncoderArgs>,
> = embassy_sync::once_lock::OnceLock::new();

// Global static for power sensors args (moved out of RTIC Local resources)
// Using OnceLock with Mutex since PowerSensorArgs needs interior mutability
// Switched from NoopRawMutex to CriticalSectionRawMutex for the I2C bus reference
// so that PowerSensorArgs implements Send+Sync and can be used in global statics
#[cfg(feature = "power_sensors")]
static POWER_SENSORS: embassy_sync::once_lock::OnceLock<
    Mutex<CriticalSectionRawMutex, PowerSensorArgs>,
> = embassy_sync::once_lock::OnceLock::new();

// CAN sleep pin - kept in Local due to OutputPin not implementing Sync
// (only accessed by ignition_task, no concurrency concerns)

#[app(device = crate::pac, peripherals = false, dispatchers = [USART1, USART2, USART3])]
mod app {

    //use embassy_stm32::can::BusError;

    use embassy_stm32::gpio::Output;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::{Channel, Receiver};
    use embassy_time::Delay;
    use embedded_hal_async::delay::DelayNs;
    use j1939_async as j1939;

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        //can_iface: can::BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>,
        //cansleep moved to ignition_task via spawn argument
        //rtc: Rtc,
        //app: crate::application::MonitorApp,
        //#[cfg(feature = "power_sensors")]
        //power_sensors: PowerSensorArgs,
        //i2c_devices: MonitorInterfaces,
        //mon_alert_pins: bsp::MonitorAlertPins,
        //mon_obs_event_sender: Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
        //can_suspended_event_sender: Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
        //#[cfg(feature = "terminal")]
        //encoder_args: EncoderArgs,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        //let mono_token = rtic_monotonics::create_systick_token!();

        #[allow(unused_variables, unused_mut)]
        let bsp::Bsp(
            device_id,
            mut can_iface,
            cansleep,
            mut ignition_pin,
            mut leds,
            nvstore_i2c,
            mut sensors_i2c,
            mon_alert_pins,
            encoder_pins,
            flash_resources,
            crc,
            display_connector,
        ) = bsp::Bsp::new();

        // Initialize ignition pin as global static (moved out of RTIC Local)
        #[cfg(feature = "power_sensors")]
        {
            let _ignition_pin_ref = IGNITION_PIN.get_or_init(|| Mutex::new(ignition_pin));
        }

        leds[0].set_high();
        leds[1].set_low();
        //cansleep.set_low();

        // Setup CAN
        can_iface.properties().set_extended_filter(
            can::filter::ExtendedFilterSlot::_0,
            can::filter::ExtendedFilter::accept_all_into_fifo1(),
        );

        // 250k bps
        can_iface.set_bitrate(250_000);

        let can_iface = can_iface.start(can::OperatingMode::NormalOperationMode);
        static TX_BUF: StaticCell<can::TxBuf<CAN_TX_BUF_SIZE>> = StaticCell::new();
        static RX_BUF: StaticCell<can::RxBuf<CAN_RX_BUF_SIZE>> = StaticCell::new();
        let can_iface = can_iface.buffered(
            TX_BUF.init(can::TxBuf::<CAN_TX_BUF_SIZE>::new()),
            RX_BUF.init(can::RxBuf::<CAN_RX_BUF_SIZE>::new()),
        );
        // Get reader/writer before moving can_iface into static
        let can_reader = can_iface.reader();
        let can_writer = can_iface.writer();
        let mut error_sender = crate::bsp::BufferedCanErrorSender::new(can_writer.clone());
        // Initialize CAN interface as global static (moved out of RTIC Local)
        CAN_IFACE.get_or_init(|| can_iface);

        error_sender.report(FILE_CODE, ErrorCode::CheckPoint as u8, line!());
        // Initialize main error sender as global static (moved out of RTIC Local)
        MAIN_ERROR_SENDER.get_or_init(|| Mutex::new(error_sender.clone()));

        let _eeprom = {
            use eeprom24x::{Eeprom24x, SlaveAddr};
            let address = SlaveAddr::default();
            let eeprom = Eeprom24x::new_24x256(nvstore_i2c, address);

            let delay = embassy_time::Delay {};

            eeprom24x::Storage::new(eeprom, delay)
        };
        leds[1].set_high();

        #[cfg(feature = "power_sensors")]
        let mut addresses = {
            type Addresses = heapless::Vec<u8, { consts::MAX_MONITORS }>;
            Addresses::new()
        };

        // Setup I2C Bus manager (async I2c, blocking methods work during init)
        #[cfg(feature = "power_sensors")]
        static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, bsp::SensorI2c>> =
            StaticCell::new();
        #[cfg(feature = "power_sensors")]
        let i2c_manager = {
            // Scan for I2C addresses using blocking methods (works on async I2c too).
            for addr in bsp::consts::INA226_ADDRS {
                let mut dummy = [0u8; 0];
                let dummy2 = [0u8; 0];
                match sensors_i2c.blocking_write_read(addr, &dummy2, &mut dummy) {
                    Ok(()) => {
                        //defmt::println!("I2C addr={}", addr);
                        addresses.push(addr).unwrap();
                    }
                    Err(_e) => {}
                }
            }
            I2C_BUS.init(Mutex::new(sensors_i2c))
        };
        //#[cfg(not(feature = "power_sensors"))]
        //pub type I2cDeviceIf = I2cDevice<'static, NoopRawMutex, bsp::SensorI2c>;
        //use bme280::i2c::BME280;
        //let bme = BME280::new_primary(
        //    I2cDeviceIf::new(i2c_manager),
        //    crate::application::DelayForBme280 {},
        //);

        // Setup I2C devices (create wrappers only, no I2C traffic yet - RTIC init is sync)
        #[cfg(feature = "power_sensors")]
        let i2c_devices = {
            let mut i2c_devices = MonitorInterfaces::new();
            for addr in addresses {
                if i2c_devices
                    .push(MonitorInterface {
                        chip: INA226::new(I2cDevice::new(i2c_manager), addr),
                    })
                    .is_err()
                {
                    error_sender.report(FILE_CODE, crate::ErrorCode::NoDevice as u8, line!());
                }
            }
            i2c_devices
        };

        #[cfg(feature = "terminal")]
        let (display, display_reset_pin) = {
            let st = embassy_time::Delay;
            let spi = embedded_hal_bus::spi::ExclusiveDevice::new(
                display_connector.spi,
                display_connector.cs,
                st,
            );
            let interface = display_interface_spi::SPIInterface::new(spi, display_connector.dc);

            let raw_disp = oled_async::Builder::new(crate::application::SpecifficDisplay {})
                .with_rotation(crate::application::DISPLAY_ROTATION)
                .connect(interface);

            let display: oled_async::mode::graphics::GraphicsMode<
                _,
                _,
                { crate::application::DISPLAY_BUFFFER_SIZE },
            > = raw_disp.into();

            (display, display_connector.rst)
        };

        //let mut monitors = crate::powercalc::Monitors::new(&NVSTORE);
        //match monitors.setup(i2c_devices.len(), embassy_time::Instant::now()) {
        //    Err(e) => error_sender.send(&e),
        //    _ => {}
        //}
        leds[4].set_low();

        let app = crate::application::MonitorApp::new(
            embassy_time::Instant::now(),
            can_reader,
            can_writer,
            device_id,
            #[cfg(feature = "terminal")]
            display,
            #[cfg(feature = "terminal")]
            display_reset_pin,
            &APPDATA,
        );
        // Initialize app as global static (moved out of RTIC Local)
        APP.get_or_init(|| Mutex::new(app));

        static MAIN_CHANNEL: StaticCell<
            Channel<CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
        > = StaticCell::new();
        let main_channel = MAIN_CHANNEL.init(Channel::new());
        let main_event_sender = main_channel.sender();
        // Initialize can suspended event sender as global static (moved out of RTIC Local)
        CAN_SUSPENDED_SENDER.get_or_init(|| main_event_sender.clone());
        let main_event_receiver = main_channel.receiver();

        let nvs = nvstore::NvStore::new(
            flash_resources,
            #[cfg(feature = "power_sensors")]
            crc,
        );

        //cx.core.SCB.set_sleepdeep();

        let args = MainArgs {
            leds,
            #[cfg(feature = "power_sensors")]
            ndevices: i2c_devices.len(),
            nvs,
        };

        // Initialize encoder args as global static (moved out of RTIC Local)
        #[cfg(feature = "terminal")]
        {
            let _encoder_args = EncoderArgs(
                encoder_pins,
                main_event_sender.clone(),
                error_sender.clone(),
            );
            ENCODER_ARGS.get_or_init(|| Mutex::new(_encoder_args));
        }

        // Initialize power sensors as global static (moved out of RTIC Local)
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

        if let Err(_) = main_task::spawn(args, main_event_receiver) {
            error_sender.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }

        #[cfg(feature = "terminal")]
        match encoder_task::spawn() {
            Ok(_) => {}
            Err(_) => {
                error_sender.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }
        }

        // Spawn ignition_task with cansleep as argument (moved from main_task)
        match ignition_task::spawn(cansleep) {
            Ok(_) => {}
            Err(_) => {
                error_sender.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }
        }

        (
            // Return Shared resources
            Shared {},
            // Return Local resources
            Local {
                //can_iface,
                //cansleep moved to ignition_task via spawn argument above
                //rtc,
                //app,
                //#[cfg(feature = "power_sensors")]
                //power_sensors: PowerSensorArgs(
                //    i2c_manager,
                //    i2c_devices,
                //    mon_alert_pins,
                //    main_event_sender.clone(),
                //),
                //i2c_devices,
                //mon_alert_pins,
                //mon_obs_event_sender: main_event_sender.clone(),
                //#[cfg(feature = "terminal")]
                //encoder_args: EncoderArgs(
                //    encoder_pins,
                //    main_event_sender.clone(),
                //    error_sender.clone(),
                //),

            },
        )
    }

    #[idle(local = [x: u32 = 0])]
    fn idle(cx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        let _x: &'static mut u32 = cx.local.x;

        loop {
            // Now Wait For Interrupt is used instead of a busy-wait loop
            // to allow MCU to sleep between interrupts
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::wfi()
        }
    }

    #[task(priority = 1)]
    async fn start_flash(_cx: start_flash::Context, shared_nvs: &'static nvstore::SharedNvStore) {
        #[cfg(feature = "power_sensors")]
        loop {
            let mut header = nvstore::power_sensors::Header::new();
            Delay {}.delay_ms(60000).await;
            let now = embassy_time::Instant::now();
            defmt::println!("Try Store observation");

            header.time += 10;
            match shared_nvs
                .store_monitor_observation(&APPDATA, now, &header)
                .await
            {
                Ok(()) => {}
                Err(e) => {
                    defmt::println!("Error storing Mon Data {:?}", e);
                }
            };
        }
        #[cfg(not(feature = "power_sensors"))]
        {
            let _nvs = shared_nvs;
        }
    }

    #[task(priority = 2)]
    async fn blink(_cx: blink::Context, led: &mut crate::bsp::OutputPin) {
        let mut delay = Delay {};
        loop {
            led.set_high();
            delay.delay_ms(5).await;

            led.set_low();
            let ignition_state = IGNITION_STATE.lock().await;
            if *ignition_state {
                delay.delay_ms(1000).await;
            } else {
                delay.delay_ms(60000).await;
            }
        }
    }

    #[task(priority = 1)]
    #[allow(unused_mut)]
    async fn ignition_task(_cx: ignition_task::Context, mut cansleep: Output<'static>) {
        #[cfg(feature = "power_sensors")]
        {
            let mut enabled = true;
            defmt::println!("Ignition task started");
            let mut delay = Delay {};
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
        #[cfg(not(feature = "power_sensors"))]
        {
            let _cansleep = &mut cansleep;
        }
    }

    async fn init_storage(
        shared_nvs: &nvstore::SharedNvStore,
        leds: &mut crate::bsp::Leds,
    ) -> Result<Option<i64>, j1939::error::Error> {
        leds[0].set_low();
        let mut unlocked = shared_nvs.nv.lock().await;
        let nvs = unlocked
            .as_mut()
            .ok_or(error::mkerr(FILE_CODE, ErrorCode::NoDevice, line!()))?;

        nvs.init().await?;

        leds[1].set_low();

        //nvs.load_monitor_settings().await?;

        leds[2].set_high();

        #[cfg(feature = "power_sensors")]
        let ret = {
            let mut header = nvstore::power_sensors::Header::new();
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

    #[task(priority = 2)]
    async fn main_task(
        _cx: main_task::Context,
        mut args: MainArgs,
        events: Receiver<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
    ) {
        *(NVSTORE.nv.lock().await) = Some(args.nvs);

        #[cfg(feature = "power_sensors")]
        {
            let mut unlocked = APPDATA.lock().await;
            match unlocked
                .monitors
                .setup(args.ndevices, embassy_time::Instant::now())
                .await
            {
                Err(e) => {
                    let mut guard = MAIN_ERROR_SENDER.get().await.lock().await;
                    guard.send(&e);
                }
                _ => {}
            }
        }

        //nvs.erase_pd().await.unwrap();

        match init_storage(&NVSTORE, &mut args.leds).await {
            Ok(Some(_time)) => {
                /*let unlocked = data_store.lock().await;
                defmt::println!(
                    "Loaded from NV Time {} {} {} {} {}",
                    time,
                    unlocked.monitors.monitor(0).total_charge_ua_ms(),
                    unlocked.monitors.monitor(1).total_charge_ua_ms(),
                    unlocked.monitors.monitor(2).total_charge_ua_ms(),
                    unlocked.monitors.monitor(3).total_charge_ua_ms()
                );*/
            }
            Ok(None) => {}
            Err(e) => {
                {
                    let mut guard = MAIN_ERROR_SENDER.get().await.lock().await;
                    guard.send(&e);
                }
                //defmt::println!("Failed to initialize storage: {:?}", e);
            }
        }

        #[cfg(feature = "power_sensors")]
        if let Err(_) = i2c_task::spawn() {
            {
                let mut guard = MAIN_ERROR_SENDER.get().await.lock().await;
                guard.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }
        }

        if let Err(_) = start_flash::spawn(&NVSTORE) {
            {
                let mut guard = MAIN_ERROR_SENDER.get().await.lock().await;
                guard.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }
        }

        args.leds[4].set_high();

        if let Err(_) = blink::spawn(&mut args.leds[0]) {
            {
                let mut guard = MAIN_ERROR_SENDER.get().await.lock().await;
                guard.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }
        }

        {
            let mut guard = APP.get().await.lock().await;
            match guard.init().await {
                Err(err) => {
                    let mut error_guard = MAIN_ERROR_SENDER.get().await.lock().await;
                    error_guard.send(&err);
                }
                _ => {}
            }
        }

        use embassy_futures::select::{select, Either};

        loop {
            let ret = select(events.receive(), async {
                let mut guard = APP.get().await.lock().await;
                if let Err(e) = guard.run().await {
                    {
                        let mut error_guard = MAIN_ERROR_SENDER.get().await.lock().await;
                        error_guard.send(&e);
                    }
                }
            })
            .await;

            match ret {
                Either::First(event) => {
                    let mut guard = APP.get().await.lock().await;
                    guard.on_event(&event).await;
                }
                Either::Second(()) => {}
            }
        }
    }

    #[allow(unused_mut, unused_variables)]
    #[task(priority = 3)]
    async fn i2c_task(mut cx: i2c_task::Context) {
        #[cfg(feature = "power_sensors")]
        {
            let mut guard = POWER_SENSORS.get().await.lock().await;

            // Discover and configure INA226 devices, then enter the alert-driven monitoring loop
            crate::sensors::init_power_sensors(&mut *guard).await;
            crate::sensors::run_power_sensors(&mut *guard).await;
        }
    }

    #[task(priority = 1)]
    async fn encoder_task(cx: encoder_task::Context) {
        //let EncoderArgs(mut pins, mut sender, mut error_sender) = cx.local.encoder_args;
        //run_encoder(&mut pins, &mut sender, &mut error_sender).await;
        //let _dr = &cx.shared.dummy;
        #[cfg(feature = "terminal")]
        {
            let mut guard = ENCODER_ARGS.get().await.lock().await;
            run_encoder(&mut *guard).await;
        }
        // Avoid unused variable warning
        let _cx = &cx;
    }

    #[cfg(feature = "terminal")]
    async fn run_encoder(
        //pins: &mut crate::bsp::InputPins,
        //sender: &mut Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
        //error_sender: &mut crate::bsp::BufferedCanErrorSender,
        args: &mut EncoderArgs,
    ) {
        //let mut delay = Delay {};
        //delay.delay_ms(1000).await;

        let EncoderArgs(pins, sender, error_sender) = args;
        let [button, enc_a, enc_b] = pins;

        // Debounce the inputs
        let mut enc_a =
            async_debounce::Debouncer::new(enc_a, embassy_time::Duration::from_micros(10));
        let mut enc_b =
            async_debounce::Debouncer::new(enc_b, embassy_time::Duration::from_micros(10));
        let mut button =
            async_debounce::Debouncer::new(button, embassy_time::Duration::from_micros(100));

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
            use crate::application::MainEvent;
            use embassy_futures::select::{select3, Either3};
            use embedded_hal::digital::InputPin;
            use embedded_hal_async::digital::Wait;
            let res: Result<(), Error> = match select3(
                enc_a.wait_for_any_edge(),
                enc_b.wait_for_any_edge(),
                button.wait_for_any_edge(),
            )
            .await
            {
                // For Encoder A and B. Just look for A,B raising edges or B,A raising edges. Any falling edge, resets state to idle.
                Either3::First(_) => match enc_a.is_high() {
                    Ok(true) => {
                        if encoder_state == EncoderState::B {
                            encoder_state = EncoderState::Idle;
                            sender
                                .send(MainEvent::HID(HumanEvent::EncoderAntiClockwise))
                                .await;
                            Ok(())
                        } else {
                            encoder_state = EncoderState::A;
                            Ok(())
                        }
                    }
                    Ok(false) => {
                        encoder_state = EncoderState::Idle;
                        Ok(())
                    }
                    Err(_) => {
                        encoder_state = EncoderState::Idle;
                        Ok(())
                    }
                },
                Either3::Second(_) => match enc_b.is_high() {
                    Ok(true) => {
                        if encoder_state == EncoderState::A {
                            encoder_state = EncoderState::Idle;
                            sender
                                .send(MainEvent::HID(HumanEvent::EncoderClockwise))
                                .await;
                            Ok(())
                        } else {
                            encoder_state = EncoderState::B;
                            Ok(())
                        }
                    }
                    Ok(false) => {
                        encoder_state = EncoderState::Idle;
                        Ok(())
                    }
                    Err(_) => {
                        encoder_state = EncoderState::Idle;
                        Ok(())
                    }
                },
                Either3::Third(_) => match button.is_high() {
                    Ok(true) => {
                        press_time = Some(embassy_time::Instant::now());
                        sender
                            .send(MainEvent::HID(HumanEvent::EncoderButtonPressed))
                            .await;
                        Ok(())
                    }
                    Ok(false) => {
                        if press_time.is_none() {
                            // Spurious?
                            Ok(())
                        } else {
                            let duration = embassy_time::Instant::now() - press_time.unwrap();
                            sender
                                .send(MainEvent::HID(HumanEvent::EncoderButtonReleased(duration)))
                                .await;
                            Ok(())
                        }
                    }
                    Err(_) => {
                        encoder_state = EncoderState::Idle;
                        Ok(())
                    }
                },
            };
            match res {
                Ok(_) => {}
                Err(e) => {
                    error_sender.send(&e);
                }
            }
        }
    }
}
