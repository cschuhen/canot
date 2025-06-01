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
pub mod types;
#[cfg(feature = "terminal")]
pub mod ui;

#[cfg(feature = "power_sensors")]
use core::cell::RefCell;
#[cfg(feature = "power_sensors")]
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_stm32::can;
#[cfg(feature = "power_sensors")]
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
#[cfg(feature = "power_sensors")]
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Delay;
use rtic_sync::{channel::*, make_channel};
//use embassy_sync::channel::*;
use static_cell::StaticCell;

#[cfg(feature = "power_sensors")]
use ina226::INA226;
#[cfg(feature = "power_sensors")]
type MonitorChip = INA226<bsp::SensorDevice>;

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
type MonitorInterfaces = heapless::Vec<MonitorInterface, { consts::MAX_MONITORS }>;

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
    Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
    crate::bsp::BufferedCanErrorSender,
);

#[cfg(feature = "power_sensors")]
struct PowerSensorArgs(
    MonitorInterfaces,
    bsp::MonitorAlertPins,
    Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
);

static NVSTORE: crate::nvstore::SharedNvStore = crate::nvstore::SharedNvStore::new();

#[app(device = crate::pac, peripherals = false, dispatchers = [USART1, USART2, USART3])]
mod app {

    //use embassy_stm32::can::BusError;

    use embedded_hal_async::{delay::DelayNs, digital::Wait};

    use super::*;

    #[shared]
    struct Shared {
        data_store: &'static application::DataStore,
        ignition_state: bool,
    }

    #[local]
    struct Local {
        main_error_sender: crate::bsp::BufferedCanErrorSender,
        can_iface: can::BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>,
        cansleep: crate::bsp::OutputPin,
        ignition_pin: crate::bsp::ExtiPin,
        //rtc: Rtc,
        app: crate::application::MonitorApp,
        #[cfg(feature = "power_sensors")]
        power_sensors: PowerSensorArgs,
        //i2c_devices: MonitorInterfaces,
        //mon_alert_pins: bsp::MonitorAlertPins,
        //mon_obs_event_sender: Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
        can_suspended_event_sender: Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
        #[cfg(feature = "terminal")]
        encoder_args: EncoderArgs,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        //let mono_token = rtic_monotonics::create_systick_token!();

        #[allow(unused_variables, unused_mut)]
        let bsp::Bsp(
            device_id,
            mut can_iface,
            cansleep,
            ignition_pin,
            mut leds,
            nvstore_i2c,
            mut sensors_i2c,
            mon_alert_pins,
            encoder_pins,
            flash_resources,
            crc,
            display_connector,
        ) = bsp::Bsp::new();

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
        let mut error_sender = crate::bsp::BufferedCanErrorSender::new(can_iface.writer());

        error_sender.report(FILE_CODE, ErrorCode::CheckPoint as u8, line!());

        static APPDATA: application::DataStore =
            embassy_sync::mutex::Mutex::new(application::Data::new(&NVSTORE));

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

        // Setup I2C Bus manager
        #[cfg(feature = "power_sensors")]
        let i2c_manager = {
            // Scan for I2C addresses.
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
            static I2C_BUS: StaticCell<Mutex<NoopRawMutex, RefCell<bsp::SensorI2c>>> =
                StaticCell::new();
            I2C_BUS.init(Mutex::new(RefCell::new(sensors_i2c)))
        };
        //#[cfg(not(feature = "power_sensors"))]
        //pub type I2cDeviceIf = I2cDevice<'static, NoopRawMutex, bsp::SensorI2c>;
        //use bme280::i2c::BME280;
        //let bme = BME280::new_primary(
        //    I2cDeviceIf::new(i2c_manager),
        //    crate::application::DelayForBme280 {},
        //);

        // Setup I2C devices
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
                    //report_error(crate::ErrorCode::NoSpace, line!());
                }
                let mon = i2c_devices.last_mut().unwrap();
                match mon.chip.die_id() {
                    Ok(_id) => {
                        let bus = mon.chip.bus_voltage_raw();
                        let shunt = mon.chip.shunt_voltage_raw();
                        defmt::println!("Found dvc id={} bus={} shunt={}", _id, bus, shunt);
                    }
                    Err(_e) => {
                        error_sender.report(FILE_CODE, crate::ErrorCode::NoDevice as u8, line!());
                        i2c_devices.pop();
                        //report_error(crate::ErrorCode::NoDevice, line!());
                    }
                }
            }

            leds[2].set_low();

            for mon in &mut i2c_devices {
                match mon.chip.die_id() {
                    Ok(_id) => {
                        defmt::println!("Found dvc {}", _id);
                    }
                    Err(_e) => {
                        defmt::println!("No dvc");

                        error_sender.report(FILE_CODE, crate::ErrorCode::NoDevice as u8, line!());
                    }
                }
            }
            leds[3].set_low();

            for mon in &mut i2c_devices {
                // Do an initial read just to check comms.
                match (mon.chip.bus_voltage_raw(), mon.chip.shunt_voltage_raw()) {
                    (Ok(_bus), Ok(_shunt)) => { /*defmt::println!("BOOT Bus={}v Shunt={}v", _bus, _shunt)*/
                    }
                    (Err(_bus), Err(_shunt)) => {
                        error_sender.report(FILE_CODE, crate::ErrorCode::I2CRead as u8, line!())
                    }
                    (Ok(_), Err(_shunt)) => {
                        error_sender.report(FILE_CODE, crate::ErrorCode::I2CRead as u8, line!())
                    }
                    (Err(_bus), Ok(_)) => {
                        error_sender.report(FILE_CODE, crate::ErrorCode::I2CRead as u8, line!())
                    }
                }

                // Configure conversions and alert.
                let mut mon_cfg = mon.chip.configuration().unwrap().unwrap();
                mon_cfg.mode = ina226::MODE::ShuntBusVoltageContinuous;
                mon_cfg.avg = ina226::AVG::_1024;
                mon_cfg.vbusct = ina226::VBUSCT::_140us;
                mon_cfg.vshct = ina226::VSHCT::_588us;
                mon.chip.set_configuration(&mon_cfg).unwrap();

                mon.chip
                    .set_mask_enable(ina226::MaskEnableFlags::CNVR)
                    .unwrap();
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
            can_iface.reader(),
            can_iface.writer(),
            device_id,
            #[cfg(feature = "terminal")]
            display,
            #[cfg(feature = "terminal")]
            display_reset_pin,
            &APPDATA,
        );

        let (main_event_sender, main_event_receiver) =
            make_channel!(MainEvent, MAIN_EVENT_CAPACITY);

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

        (
            // Return Shared resources
            Shared {
                data_store: &APPDATA,
                ignition_state: true,
            },
            // Return Local resources
            Local {
                can_iface,
                cansleep,
                ignition_pin,
                //rtc,
                app,
                #[cfg(feature = "power_sensors")]
                power_sensors: PowerSensorArgs(
                    i2c_devices,
                    mon_alert_pins,
                    main_event_sender.clone(),
                ),
                //i2c_devices,
                //mon_alert_pins,
                //mon_obs_event_sender: main_event_sender.clone(),
                can_suspended_event_sender: main_event_sender.clone(),
                #[cfg(feature = "terminal")]
                encoder_args: EncoderArgs(
                    encoder_pins,
                    main_event_sender.clone(),
                    error_sender.clone(),
                ),
                main_error_sender: error_sender,
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
    async fn start_flash(
        _cx: start_flash::Context,
        data_store: &'static crate::application::DataStore,
        shared_nvs: &'static nvstore::SharedNvStore,
    ) {
        #[cfg(feature = "power_sensors")]
        loop {
            let mut header = nvstore::power_sensors::Header::new();
            let mut delay = Delay {};
            delay.delay_ms(60000).await;
            let now = embassy_time::Instant::now();
            defmt::println!("Try Store observation");

            header.time += 10;
            match shared_nvs
                .store_monitor_observation(data_store, now, &header)
                .await
            {
                Ok(()) => {}
                Err(e) => {
                    defmt::println!("Error storing Mon Data {:?}", e);
                }
            };
            /*match nvs.store_dirty_settings().await {
                Ok(()) => {}
                Err(e) => {
                    defmt::println!("Error storing Mon Settings {:?}", e);
                }
            }*/
        }
        #[cfg(not(feature = "power_sensors"))]
        {
            let _ds = data_store;
            let _nvs = shared_nvs;
        }
    }

    #[task(priority=2, shared = [ignition_state])]
    async fn blink(mut cx: blink::Context, led: &mut crate::bsp::OutputPin) {
        let mut delay = Delay {};
        loop {
            led.set_high();
            delay.delay_ms(5).await;

            led.set_low();
            let ignition_state = cx.shared.ignition_state.lock(|is| *is);
            if ignition_state {
                delay.delay_ms(1000).await;
            } else {
                delay.delay_ms(60000).await;
            }
        }
    }

    #[task(priority=1, shared=[ignition_state], local = [ignition_pin, cansleep, can_suspended_event_sender])]
    async fn ignition_task(mut cx: ignition_task::Context) {
        let mut enabled = true;
        defmt::println!("Ignition task started");
        let mut ignition_pin = async_debounce::Debouncer::new(
            cx.local.ignition_pin,
            embassy_time::Duration::from_millis(10),
        );
        let mut delay = Delay {};
        loop {
            use embedded_hal::digital::InputPin;
            match ignition_pin.wait_for_any_edge().await {
                Ok(()) => {
                    let ignition = ignition_pin.is_high().unwrap();

                    if ignition != enabled {
                        defmt::println!("Ignition state changed: {}", ignition);
                        enabled = ignition;

                        if enabled {
                            cx.local.cansleep.set_low();
                            delay.delay_ms(100).await;
                        } else {
                            cx.local.cansleep.set_high();
                        }

                        cx.shared.ignition_state.lock(|is| *is = enabled);

                        match cx
                            .local
                            .can_suspended_event_sender
                            .send(MainEvent::CanEnabled(enabled))
                            .await
                        {
                            Ok(()) => {}
                            Err(_e) => {
                                defmt::println!("Error sending event");
                            }
                        }
                    }
                }
            }
        }
    }

    async fn init_storage(
        shared_nvs: &nvstore::SharedNvStore,
        data_store: &'static crate::application::DataStore,
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
                .load_last_monitor_observation(data_store, -1, &mut header)
                .await?
            {
                true => Some(header.time),
                false => None,
            }
        };
        #[cfg(not(feature = "power_sensors"))]
        let ret: Option<i64> = {
            let _ds = data_store;
            None
        };

        leds[3].set_high();

        Ok(ret)
    }

    #[task(priority=2, local = [main_error_sender, can_iface, app], shared=[data_store])]
    async fn main_task(
        mut cx: main_task::Context,
        mut args: MainArgs,
        mut events: Receiver<'static, MainEvent, MAIN_EVENT_CAPACITY>,
    ) {
        let data_store = cx.shared.data_store.lock(|shared| *shared);

        *(NVSTORE.nv.lock().await) = Some(args.nvs);

        #[cfg(feature = "power_sensors")]
        {
            let mut unlocked = data_store.lock().await;
            match unlocked
                .monitors
                .setup(args.ndevices, embassy_time::Instant::now())
                .await
            {
                Err(e) => cx.local.main_error_sender.send(&e),
                _ => {}
            }
        }

        //nvs.erase_pd().await.unwrap();

        match init_storage(&NVSTORE, data_store, &mut args.leds).await {
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
                cx.local.main_error_sender.send(&e);
                //defmt::println!("Failed to initialize storage: {:?}", e);
            }
        }

        if let Err(_) = i2c_task::spawn() {
            cx.local
                .main_error_sender
                .report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }

        if let Err(_) = start_flash::spawn(data_store, &NVSTORE) {
            cx.local
                .main_error_sender
                .report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }

        args.leds[4].set_high();

        if let Err(_) = blink::spawn(&mut args.leds[0]) {
            cx.local
                .main_error_sender
                .report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
        }

        match cx.local.app.init().await {
            Err(err) => {
                cx.local.main_error_sender.send(&err);
            }
            _ => {}
        }

        // Schedule the blinking task
        match ignition_task::spawn() {
            Ok(_) => {}
            Err(_) => {
                cx.local
                    .main_error_sender
                    .report(FILE_CODE, ErrorCode::SpawnError as u8, line!());
            }
        }

        use embassy_futures::select::{select, Either};

        loop {
            let ret = select(events.recv(), cx.local.app.run()).await;

            match ret {
                Either::First(Ok(event)) => {
                    cx.local.app.on_event(&event).await;
                }
                Either::First(Err(rtic_err)) => {
                    cx.local.main_error_sender.reportd(
                        FILE_CODE,
                        ErrorCode::RunFail as u8,
                        line!(),
                        rtic_err as u32,
                    );
                }
                Either::Second(Ok(())) => {}
                Either::Second(Err(e)) => {
                    cx.local.app.on_error(e);
                }
            }
        }
    }

    #[cfg(feature = "power_sensors")]
    fn read_monitor(
        chip: &mut MonitorChip,
        index: u8,
        event_sender: &mut Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
    ) {
        // Must read this to clear alert.
        let mon_mask = chip.mask_enable().unwrap();

        if !mon_mask.contains(ina226::MaskEnableFlags::CVRF) {
            return;
        }

        let bus = chip.bus_voltage_raw();
        let shunt = chip.shunt_voltage_raw();
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

    #[task(priority=3,
        local = [power_sensors
            ])]
    async fn i2c_task(cx: i2c_task::Context) {
        #[cfg(feature = "power_sensors")]
        run_power_sensors(cx.local.power_sensors).await;
        // Avoid unused variable warning
        let _cx = &cx;
    }

    #[cfg(feature = "power_sensors")]
    async fn run_power_sensors(
        //pins: &mut crate::bsp::InputPins,
        //sender: &mut Sender<'static, MainEvent, MAIN_EVENT_CAPACITY>,
        //error_sender: &mut crate::bsp::BufferedCanErrorSender,
        args: &mut PowerSensorArgs,
    ) {
        //let mut delay = Delay {};
        //delay.delay_ms(1000).await;

        use embassy_futures::select::{select4, Either4};
        let PowerSensorArgs(devices, alert_pins, event_sender) = args;
        let [m0, m1, m2, m3] = alert_pins;

        loop {
            //let futures = cx.local.mon_alert_pins.map(|p| p.wait_for_low());
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
            //defmt::println!("GotAlert {}/{}", index, cx.local.i2c_devices.len());
            //cx.local.mon0_alert_pin.wait_for_low().await;
            read_monitor(&mut devices[index].chip, index as u8, event_sender);
            //defmt::println!("DoneAlert {}/{}", index, cx.local.i2c_devices.len());
        }
    }
    //#[cfg(feature = "terminal")]

    #[task(priority=1, local = [encoder_args])]
    async fn encoder_task(cx: encoder_task::Context) {
        //let EncoderArgs(mut pins, mut sender, mut error_sender) = cx.local.encoder_args;
        //run_encoder(&mut pins, &mut sender, &mut error_sender).await;
        //let _dr = &cx.shared.dummy;
        #[cfg(feature = "terminal")]
        run_encoder(cx.local.encoder_args).await;
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
        let mut encoder_state: EncoderState = EncoderState::Idle;
        loop {
            use crate::application::HumanEvent;
            use crate::application::MainEvent;
            use embassy_futures::select::{select3, Either3};
            use embedded_hal::digital::InputPin;
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
                                .await
                                .map_err(|e| {
                                    j1939::error::mkerr_generic(
                                        FILE_CODE,
                                        crate::error::ErrorCode::SendEvent as u8,
                                        line!(),
                                        e,
                                    )
                                })
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
                                .await
                                .map_err(|e| {
                                    j1939::error::mkerr_generic(
                                        FILE_CODE,
                                        crate::error::ErrorCode::SendEvent as u8,
                                        line!(),
                                        e,
                                    )
                                })
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
                    Ok(true) => sender
                        .send(MainEvent::HID(HumanEvent::EncoderButtonPressed))
                        .await
                        .map_err(|e| {
                            j1939::error::mkerr_generic(
                                FILE_CODE,
                                crate::error::ErrorCode::SendEvent as u8,
                                line!(),
                                e,
                            )
                        }),
                    Ok(false) => sender
                        .send(MainEvent::HID(HumanEvent::EncoderButtonReleased))
                        .await
                        .map_err(|e| {
                            j1939::error::mkerr_generic(
                                FILE_CODE,
                                crate::error::ErrorCode::SendEvent as u8,
                                line!(),
                                e,
                            )
                        }),
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
