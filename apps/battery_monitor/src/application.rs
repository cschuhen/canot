use crate::error::*;
use embassy_stm32::can;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use j1939::can::CanInterface;
use j1939_async as j1939;
use j1939_async::Id;
use oled_async::display::DisplayVariant;
use oled_async::prelude::*;

#[cfg(feature = "power_sensors")]
use crate::powercalc::Monitors;

const FILE_CODE: u8 = 0x02;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum HumanEvent {
    EncoderClockwise,
    EncoderAntiClockwise,
    EncoderButtonPressed,
    EncoderButtonReleased,
}

pub enum MainEvent {
    //Init,
    //Timeout,
    Error(crate::Error),
    #[cfg(feature = "power_sensors")]
    MonitorObservation(crate::powercalc::Observation),
    CanEnabled(bool),
    #[cfg(feature = "terminal")]
    HID(HumanEvent),
    //CanRx,
}

fn j1939_name(device_id: u32) -> j1939::name::Name {
    j1939::name::Name::create(true, 0, 0, 0, 0, 0, 0, 0, device_id)
}

// Boilerplate shit so we can have a display.
pub type DisplayDevice = embedded_hal_bus::spi::ExclusiveDevice<
    crate::bsp::Spi,
    crate::bsp::OutputPin,
    embassy_time::Delay,
>;
type DisplayInterface = display_interface_spi::SPIInterface<DisplayDevice, crate::bsp::OutputPin>;
#[cfg(not(feature = "sh1108_128_160"))]
pub type SpecifficDisplay = oled_async::displays::ssd1309::Ssd1309_128_64;
#[cfg(not(feature = "sh1108_128_160"))]
pub const DISPLAY_ROTATION: oled_async::displayrotation::DisplayRotation =
    oled_async::displayrotation::DisplayRotation::Rotate180;
#[cfg(feature = "sh1108_128_160")]
pub type SpecifficDisplay = oled_async::displays::sh1108::Sh1108_128_160;
#[cfg(feature = "sh1108_128_160")]
pub const DISPLAY_ROTATION: oled_async::displayrotation::DisplayRotation =
    oled_async::displayrotation::DisplayRotation::Rotate270;
pub const DISPLAY_BUFFFER_SIZE: usize =
    SpecifficDisplay::WIDTH as usize * SpecifficDisplay::HEIGHT as usize / 8;
pub type Display = GraphicsMode<SpecifficDisplay, DisplayInterface, { DISPLAY_BUFFFER_SIZE }>;
pub type ResetPin = embassy_stm32::gpio::Output<'static>;

pub struct Data {
    #[cfg(feature = "power_sensors")]
    pub monitors: crate::powercalc::Monitors,
}

impl Data {
    pub const fn new(nv: &'static crate::nvstore::SharedNvStore) -> Self {
        {
            let mut _nv = nv;
        }
        Data {
            #[cfg(feature = "power_sensors")]
            monitors: Monitors::new(nv),
        }
    }
}

pub type DataStore = Mutex<CriticalSectionRawMutex, crate::application::Data>;

struct CanSender {
    queue: can::BufferedCanSender,
}

impl CanSender {
    fn new(queue: can::BufferedCanSender) -> Self {
        CanSender { queue }
    }
}

impl CanInterface for CanSender {
    fn transmit(&mut self, frame: j1939::can::Frame) -> Result<(), j1939::can::Frame> {
        use embassy_stm32::can::frame::Frame;
        match Frame::new_extended(frame.id().as_raw(), frame.data().as_slice()) {
            Ok(cf) => {
                self.queue.try_write(cf).map_err(|x| match x {
                    embassy_sync::channel::TrySendError::<Frame>::Full(_pkt) => {
                        //defmt::println!("No Receiver");
                        frame
                    }
                })
            }
            Err(_) => Ok(()),
        }
    }
}

pub struct MonitorApp {
    error_sender: crate::bsp::BufferedCanErrorSender,
    can_receiver: can::BufferedCanReceiver,
    can_sender: CanSender,
    stack: j1939::stack::SharedStack,
    time: crate::types::TimeInstant,
    name: j1939::name::Name,
    pub data_store: &'static crate::application::DataStore,
    #[cfg(feature = "terminal")]
    ui: crate::ui::Ui,
} // struct MonitorApp

impl MonitorApp {
    pub fn new(
        time: crate::types::TimeInstant,
        can_receiver: can::BufferedCanReceiver,
        can_sender: can::BufferedCanSender,
        device_id: u32,
        #[cfg(feature = "terminal")] display: Display,
        #[cfg(feature = "terminal")] display_reset_pin: ResetPin,
        data_store: &'static crate::application::DataStore,
    ) -> MonitorApp {
        defmt::println!("Device ID {:x}", device_id);
        MonitorApp {
            error_sender: crate::bsp::BufferedCanErrorSender::new(can_sender.clone()),
            can_receiver,
            can_sender: CanSender::new(can_sender),
            stack: j1939::stack::SharedStack::new(),
            time,
            name: j1939_name(device_id),
            data_store,
            #[cfg(feature = "terminal")]
            ui: crate::ui::Ui::new(display, display_reset_pin),
        }
    }

    pub async fn init(&mut self) -> Result<bool, Error> {
        //self.nvstore.test();

        match self
            .stack
            .register_local_name(&mut self.can_sender, 0x90, &self.name)
            .await
        {
            Ok(_) => Ok(true),
            Err(_) => Err(crate::error::mkerr(FILE_CODE, ErrorCode::CAN, line!())),
        }?;
        #[cfg(feature = "terminal")]
        self.init_display().await?;
        Ok(true)
    }

    #[cfg(feature = "terminal")]
    pub async fn init_display(&mut self) -> Result<(), Error> {
        use crate::ui;
        use j1939::process_data::{DDI_PROP_CURRENT, DDI_PROP_TOTAL_CHARGE, DDI_PROP_VOLTAGE};

        let mut st = embassy_time::Delay;
        self.ui.reset_display(&mut st)?;
        self.ui.display.init().await.map_err(|e| {
            j1939::error::mkerr_generic(
                FILE_CODE,
                crate::error::ErrorCode::DisplayDraw as u8,
                line!(),
                e,
            )
        })?;

        self.ui.add_pd_number_with_icon_and_unit(
            10,
            DDI_PROP_TOTAL_CHARGE,
            ui::SOLAR,
            &ui::UNIT_CAPACITY,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            11,
            DDI_PROP_TOTAL_CHARGE,
            ui::BATTERY,
            &ui::UNIT_CAPACITY,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            12,
            DDI_PROP_TOTAL_CHARGE,
            ui::BATTERY,
            &ui::UNIT_CAPACITY,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            13,
            DDI_PROP_TOTAL_CHARGE,
            ui::LIGHT,
            &ui::UNIT_CAPACITY,
        )?;

        self.ui.add_pd_number_with_icon_and_unit(
            10,
            DDI_PROP_CURRENT,
            ui::SOLAR,
            &ui::UNIT_CURRENT,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            11,
            DDI_PROP_CURRENT,
            ui::BATTERY,
            &ui::UNIT_CURRENT,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            12,
            DDI_PROP_CURRENT,
            ui::BATTERY,
            &ui::UNIT_CURRENT,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            13,
            DDI_PROP_CURRENT,
            ui::LIGHT,
            &ui::UNIT_CURRENT,
        )?;

        self.ui.add_pd_number_with_icon_and_unit(
            10,
            DDI_PROP_VOLTAGE,
            ui::SOLAR,
            &ui::UNIT_VOLT,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            11,
            DDI_PROP_VOLTAGE,
            ui::BATTERY,
            &ui::UNIT_VOLT,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            12,
            DDI_PROP_VOLTAGE,
            ui::BATTERY,
            &ui::UNIT_VOLT,
        )?;
        self.ui.add_pd_number_with_icon_and_unit(
            13,
            DDI_PROP_VOLTAGE,
            ui::LIGHT,
            &ui::UNIT_VOLT,
        )?;

        //self.display.set_contrast(16).await.unwrap();

        self.ui.redraw().await?;

        Ok(())
    }

    async fn advance_time(&mut self) -> Result<bool, Error> {
        let time = embassy_time::Instant::now();
        let tdelta = time - self.time;
        let tdelta_ms = tdelta.as_millis();
        self.stack
            .advance_time(&mut self.can_sender, tdelta_ms as usize)
            .await?;

        self.time = time;
        Ok(true)
    }

    pub fn on_error(&mut self, error: crate::Error) {
        self.error_sender.send(&error);
    }

    #[cfg(feature = "terminal")]
    pub async fn on_hid(&mut self, event: HumanEvent) -> Result<bool, Error> {
        use crate::ui::ButtonMode;
        let contrast_step: i16 = 20;
        match event {
            HumanEvent::EncoderClockwise => {
                match self.ui.mode {
                    ButtonMode::Scroll => {
                        self.ui.selected += 1;
                    }
                    ButtonMode::X => {
                        self.ui.xoff += 3;
                    }
                    ButtonMode::Y => {
                        self.ui.yoff += 3;
                    }
                    ButtonMode::Numbers => {
                        for num in &mut self.ui.numbers {
                            match num.get_value().checked_mul(10) {
                                Some(n) => {
                                    num.set_value(n);
                                }
                                None => {}
                            }
                        }
                    }
                    ButtonMode::Contrast => {
                        self.ui.contrast += contrast_step;
                        if self.ui.contrast > 255 {
                            self.ui.contrast = 0;
                        }
                        defmt::println!("Contrast {}", self.ui.contrast);
                        self.ui
                            .display
                            .set_contrast(self.ui.contrast as u8)
                            .await
                            .map_err(|e| {
                                j1939::error::mkerr_generic(
                                    FILE_CODE,
                                    crate::error::ErrorCode::DisplayDraw as u8,
                                    line!(),
                                    e,
                                )
                            })?;
                    }
                }

                defmt::println!(
                    "Clockwise {},{} {}",
                    self.ui.xoff,
                    self.ui.yoff,
                    self.ui.selected
                );
            }
            HumanEvent::EncoderAntiClockwise => {
                match self.ui.mode {
                    ButtonMode::Scroll => {
                        self.ui.selected -= 1;
                    }
                    ButtonMode::X => {
                        self.ui.xoff -= 3;
                    }
                    ButtonMode::Y => {
                        self.ui.yoff -= 3;
                    }
                    ButtonMode::Numbers => {
                        for num in &mut self.ui.numbers {
                            num.set_value(num.get_value() / 10);
                        }
                    }
                    ButtonMode::Contrast => {
                        self.ui.contrast -= contrast_step;
                        if self.ui.contrast < 0 {
                            self.ui.contrast = 255;
                        }
                        defmt::println!("Contrast {}", self.ui.contrast);
                        self.ui
                            .display
                            .set_contrast(self.ui.contrast as u8)
                            .await
                            .map_err(|e| {
                                j1939::error::mkerr_generic(
                                    FILE_CODE,
                                    crate::error::ErrorCode::DisplayDraw as u8,
                                    line!(),
                                    e,
                                )
                            })?;
                    }
                }

                defmt::println!(
                    "Anticlockwise {},{} {}",
                    self.ui.xoff,
                    self.ui.yoff,
                    self.ui.selected
                );
            }
            HumanEvent::EncoderButtonPressed => {
                defmt::println!("Press")
            }
            HumanEvent::EncoderButtonReleased => {
                defmt::println!("Release");
                self.ui.mode.next();
                let mut i = 1;
                for num in &mut self.ui.numbers {
                    num.set_value(12345678 * i);
                    i += 1;
                }
            }
        }
        self.ui.redraw().await
    }

    #[cfg(feature = "terminal")]
    async fn on_broadcast_process_data(
        &mut self,
        _id: embedded_can::ExtendedId,
        pd: &j1939::process_data::ProcessData,
    ) -> Result<bool, Error> {
        if pd.element_number < 10 {
            return Ok(false);
        }
        if pd.element_number as usize >= 10 as usize + crate::consts::MAX_MONITORS {
            return Ok(false);
        }
        if pd.command != j1939::process_data::Command::Value {
            return Ok(false);
        }

        if self.ui.set_process_data(&pd)? {
            //defmt::println!("Set PD {} {} {}", pd.element_number, pd.ddi, pd.value);
            //self.ui.redraw().await.unwrap();
            self.ui.display.flush().await.map_err(|e| {
                j1939::error::mkerr_generic(
                    FILE_CODE,
                    crate::error::ErrorCode::DisplayDraw as u8,
                    line!(),
                    e,
                )
            })?;
            return Ok(true);
        }

        Ok(true)
    }

    async fn on_process_data(
        &mut self,
        id: embedded_can::ExtendedId,
        data: &j1939::can::FrameData,
    ) -> Result<bool, Error> {
        if data.len() < 8 {
            return Ok(true);
        }

        let pd = match j1939::process_data::ProcessData::from_data(&data[0..8]) {
            Some(pd) => pd,
            None => return Err(mkerr(FILE_CODE, ErrorCode::PdMissing, line!())),
        };

        #[cfg(feature = "terminal")]
        if id.destination() == 255 {
            return self.on_broadcast_process_data(id, &pd).await;
        }

        #[cfg(feature = "power_sensors")]
        {
            let mut unlocked = self.data_store.lock().await;
            let ret = match pd.element_number {
                x if x < crate::nvstore::ELEM_MIN_POWER_MON_CHAN0 => {
                    j1939::process_data::PdReturn::new_not_handled()
                }
                x if x < crate::nvstore::ELEM_MIN_POWER_MON_CHAN0 + 10 => {
                    unlocked.monitors.handle_process_data(&pd).await?
                }
                _ => j1939::process_data::PdReturn::new_not_handled(),
            };

            if false {
                defmt::println!(
                    "ProcessData: cmd={:?} eno={} ddi={:?} value={} ret={}",
                    pd.command,
                    pd.element_number,
                    pd.ddi,
                    pd.value,
                    ret,
                );
            }
            if ret.response().is_some() {
                let data: [u8; 8] = ret.response().unwrap().data();
                self.stack
                    .send_to_address(
                        &mut self.can_sender,
                        &self.name,
                        id.source(),
                        j1939::pgn::PROCESS_DATA,
                        7,
                        &data,
                    )
                    .await?;
            }
        }
        Ok(true)
    }

    async fn on_can_frame(
        &mut self,
        id: embedded_can::ExtendedId,
        data: &j1939::can::FrameData,
    ) -> Result<bool, Error> {
        self.stack
            .on_can_frame(&mut self.can_sender, id, data)
            .await?;
        use j1939::Id;

        match id.pgn() {
            j1939::pgn::PROCESS_DATA => return self.on_process_data(id, data).await,
            _ => {}
        }

        Ok(true)
    }

    pub async fn on_event(&mut self, event: &MainEvent) {
        match event {
            MainEvent::CanEnabled(enabled) => {
                defmt::println!("Can Enabled state changed: {}", *enabled);
                if *enabled {
                    self.stack.resume().await;
                } else {
                    self.stack.suspend().await;
                }
            }

            MainEvent::Error(error) => self.on_error(*error),
            #[cfg(feature = "power_sensors")]
            MainEvent::MonitorObservation(obs) => {
                match self.on_observation(embassy_time::Instant::now(), obs).await {
                    Ok(_) => {}
                    Err(e) => {
                        self.on_error(e);
                    }
                }
            }
            #[cfg(feature = "terminal")]
            MainEvent::HID(hid) => match self.on_hid(*hid).await {
                Ok(_) => {}
                Err(e) => {
                    self.on_error(e);
                }
            },
        };
    }

    #[cfg(feature = "power_sensors")]
    pub async fn on_observation(
        &mut self,
        time: crate::types::TimeInstant,
        obs: &crate::powercalc::Observation,
    ) -> Result<bool, crate::Error> {
        let idx = obs.monitor as usize;

        let (charge, current, voltage) = {
            let mut unlocked = self.data_store.lock().await;
            unlocked.monitors.handle_observation(time, obs)?;
            (
                unlocked.monitors.monitor(idx).total_charge_uah(),
                unlocked.monitors.monitor(idx).current_ua(),
                unlocked.monitors.monitor(idx).bus_voltage_uv(),
            )
        };

        self.stack
            .send_broadcast(
                &mut self.can_sender,
                &self.name,
                j1939::pgn::PROCESS_DATA,
                7,
                &j1939::process_data::ProcessData::new(
                    j1939::process_data::Command::Value,
                    10 + idx as u16,
                    j1939::process_data::DDI_PROP_TOTAL_CHARGE,
                    charge,
                )
                .data(),
            )
            .await?;

        self.stack
            .send_broadcast(
                &mut self.can_sender,
                &self.name,
                j1939::pgn::PROCESS_DATA,
                7,
                &j1939::process_data::ProcessData::new(
                    j1939::process_data::Command::Value,
                    10 + idx as u16,
                    j1939::process_data::DDI_PROP_CURRENT,
                    current,
                )
                .data(),
            )
            .await?;

        self.stack
            .send_broadcast(
                &mut self.can_sender,
                &self.name,
                j1939::pgn::PROCESS_DATA,
                7,
                &j1939::process_data::ProcessData::new(
                    j1939::process_data::Command::Value,
                    10 + idx as u16,
                    j1939::process_data::DDI_PROP_VOLTAGE,
                    voltage,
                )
                .data(),
            )
            .await?;

        Ok(true)
    }

    pub async fn on_can(
        &mut self,
        envelope: embassy_stm32::can::frame::Envelope,
    ) -> Result<bool, Error> {
        match envelope.frame.id() {
            embedded_can::Id::Standard(_id) => Ok(false),
            embedded_can::Id::Extended(id) => {
                /*use j1939::can::Id;
                defmt::println!(
                    "Can Rx time={} pgn={} src={} dest={} data={:x}",
                    envelope.ts,
                    id.pgn(),
                    id.source(),
                    id.destination(),
                    envelope.frame.data(),
                );*/
                let dat =
                    j1939::can::FrameData::from_slice(envelope.frame.data()).map_err(|e| {
                        j1939::error::mkerr_generic(
                            FILE_CODE,
                            crate::error::ErrorCode::PdMissing as u8,
                            line!(),
                            e,
                        )
                    })?;
                self.on_can_frame(*id, &dat).await
            }
        }
    }

    pub async fn run(&mut self) -> Result<(), Error> {
        let mut delay = embassy_time::Delay {};
        use embedded_hal_async::delay::DelayNs;
        match self.stack.get_next_timeout_ms().await {
            Some(ms) => {
                use embassy_futures::select::{select, Either};

                match select(self.can_receiver.receive(), delay.delay_ms(ms)).await {
                    Either::First(Ok(envelope)) => {
                        self.advance_time().await?;
                        self.on_can(envelope).await?;
                    }
                    Either::First(Err(_)) => {}
                    Either::Second(_) => {
                        self.advance_time().await?;
                    }
                }
            }
            None => match self.can_receiver.receive().await {
                Ok(envelope) => {
                    //self.advance_time().await?;
                    self.on_can(envelope).await?;
                }
                Err(_) => {}
            },
        }
        Ok(())
    }
}
