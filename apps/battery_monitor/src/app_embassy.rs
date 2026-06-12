#![no_std]
#![no_main]

#[allow(dead_code)]
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use crate::application::MainEvent;
use crate::error::*;
use rtic::app;

pub mod application;
pub use crate::application::MAIN_EVENT_CAPACITY;

mod bsp;
pub mod can_init;
pub mod consts;
#[cfg(feature = "terminal")]
pub mod encoder;
pub mod error;
pub mod graphical_elements;
#[cfg(feature = "power_sensors")]
pub mod ignition_input;
pub mod nvstore;
pub mod powercalc;
#[cfg(feature = "power_sensors")]
pub mod sensors;
pub mod storage;
pub mod types;
#[cfg(feature = "terminal")]
pub mod ui;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[cfg(any(feature = "power_sensors", feature = "terminal"))]
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

const FILE_CODE: u8 = 0x01;

type Error = crate::error::Error;

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

#[allow(dead_code)]
pub struct EncoderArgs(
    crate::bsp::InputPins,
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
    crate::bsp::BufferedCanErrorSender,
);

#[cfg(feature = "power_sensors")]
pub struct PowerSensorArgs(
    &'static Mutex<CriticalSectionRawMutex, bsp::SensorI2c>,
    sensors::MonitorInterfaces,
    bsp::MonitorAlertPins,
    Sender<'static, CriticalSectionRawMutex, MainEvent, MAIN_EVENT_CAPACITY>,
);

static NVSTORE: crate::nvstore::SharedNvStore = crate::nvstore::SharedNvStore::new();

// Global static for data store (moved out of RTIC Shared resources)
static APPDATA: application::DataStore =
    embassy_sync::mutex::Mutex::new(application::Data::new(&NVSTORE));

// Global static for application (moved out of RTIC Local resources)
// Using OnceLock with Mutex since MonitorApp needs interior mutability
static APP: embassy_sync::once_lock::OnceLock<
    Mutex<CriticalSectionRawMutex, crate::application::MonitorApp>,
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

/*
#[app(device = crate::pac, peripherals = false, dispatchers = [USART1, USART2, USART3])]
mod app {
*/
//use embassy_stm32::can::BusError;

//use embassy_stm32::gpio::Output;
//use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use j1939_async as j1939;

//use super::*;
/*
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

*/
/*


*/
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    //let p = embassy_stm32::init(Default::default());
    info!("Hello World!");
    /*

    */
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
        let _ignition_pin_ref =
            crate::ignition_input::IGNITION_PIN.get_or_init(|| Mutex::new(ignition_pin));
    }

    leds[0].set_high();
    leds[1].set_low();
    //cansleep.set_low();

    // Setup CAN
    let (can_receiver, can_sender, mut error_sender) = crate::can_init::init_can(can_iface);

    let _eeprom = {
        use eeprom24x::{Eeprom24x, SlaveAddr};
        let address = SlaveAddr::default();
        let eeprom = Eeprom24x::new_24x256(nvstore_i2c, address);

        let delay = embassy_time::Delay {};

        eeprom24x::Storage::new(eeprom, delay)
    };
    leds[1].set_high();

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
        can_receiver,
        can_sender,
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
    #[cfg(feature = "power_sensors")]
    {
        // Initialize can suspended event sender as global static (moved out of RTIC Local)
        crate::ignition_input::CAN_SUSPENDED_SENDER.get_or_init(|| main_event_sender.clone());
    }
    let main_event_receiver = main_channel.receiver();

    let nvs = nvstore::NvStore::new(
        flash_resources,
        #[cfg(feature = "power_sensors")]
        crc,
    );

    //cx.core.SCB.set_sleepdeep();

    // Initialize power sensors as global static (moved out of RTIC Local)
    #[cfg(feature = "power_sensors")]
    let power_sensor_args = crate::sensors::init_power_sensor_args(
        sensors_i2c,
        mon_alert_pins,
        main_event_sender.clone(),
    );

    let args = MainArgs {
        leds,
        #[cfg(feature = "power_sensors")]
        ndevices: power_sensor_args.1.len(),
        nvs,
    };

    // Initialize power sensors as global static (moved out of RTIC Local)
    #[cfg(feature = "power_sensors")]
    {
        POWER_SENSORS.get_or_init(|| Mutex::new(power_sensor_args));
    }

    /*
            if let Err(_) = main_task::spawn(args, main_event_receiver) {
                error_sender.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }

            #[cfg(feature = "terminal")]
            {
                let encoder_args = EncoderArgs(
                    encoder_pins,
                    main_event_sender.clone(),
                    error_sender.clone(),
                );
                if let Err(_) = encoder_task::spawn(encoder_args) {
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
                Local {},
            )
        }

    */

    //let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    loop {
        info!("high");
        //led.set_high();
        Timer::after_millis(1000).await;

        info!("low");
        //led.set_low();
        Timer::after_millis(1000).await;
    }
}
/*

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
        #[cfg(feature = "power_sensors")]
        {
            let ignition_state = crate::ignition_input::IGNITION_STATE.lock().await;
            if *ignition_state {
                delay.delay_ms(1000).await;
            } else {
                delay.delay_ms(60000).await;
            }
        }
        #[cfg(not(feature = "power_sensors"))]
        {
            delay.delay_ms(1000).await;
        }
    }
}

#[allow(unused_mut, unused_variables)]
#[task(priority = 1)]
async fn ignition_task(_cx: ignition_task::Context, mut cansleep: Output<'static>) {
    #[cfg(feature = "power_sensors")]
    crate::ignition_input::run_ignition(&mut cansleep).await;
}

async fn init_storage(
    shared_nvs: &nvstore::SharedNvStore,
    leds: &mut crate::bsp::Leds,
) -> Result<Option<i64>, j1939::error::Error> {
    crate::storage::run_init_storage(shared_nvs, leds).await
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
                let mut guard = crate::can_init::MAIN_ERROR_SENDER.get().await.lock().await;
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
                let mut guard = crate::can_init::MAIN_ERROR_SENDER.get().await.lock().await;
                guard.send(&e);
            }
            //defmt::println!("Failed to initialize storage: {:?}", e);
        }
    }

    #[cfg(feature = "power_sensors")]
    if let Err(_) = i2c_task::spawn() {
        {
            let mut guard = crate::can_init::MAIN_ERROR_SENDER.get().await.lock().await;
            guard.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }
    }

    if let Err(_) = start_flash::spawn(&NVSTORE) {
        {
            let mut guard = crate::can_init::MAIN_ERROR_SENDER.get().await.lock().await;
            guard.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }
    }

    args.leds[4].set_high();

    if let Err(_) = blink::spawn(&mut args.leds[0]) {
        {
            let mut guard = crate::can_init::MAIN_ERROR_SENDER.get().await.lock().await;
            guard.report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }
    }

    // App init + event loop - moved to application.rs for cleaner separation
    APP.get().await.lock().await.run_loop(events).await;
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

#[allow(unused_mut, unused_variables)]
#[task(priority = 1)]
async fn encoder_task(_cx: encoder_task::Context, mut args: EncoderArgs) {
    #[cfg(feature = "terminal")]
    crate::encoder::run_encoder(&mut args).await;
}


*/
/*
 */
/*

}


*/
