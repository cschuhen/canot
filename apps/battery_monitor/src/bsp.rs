use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode;
use embassy_stm32::peripherals::*;
use embassy_stm32::{bind_interrupts, can, i2c, peripherals, spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

bind_interrupts!(struct CanIrqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});
bind_interrupts!(struct I2c1Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

bind_interrupts!(struct I2c2Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

pub type ExtiPin = embassy_stm32::exti::ExtiInput<'static>;
pub type MonitorAlertPins = [ExtiPin; 4];
pub type InputPins = [ExtiPin; 3];
pub type NvI2c = embassy_stm32::i2c::I2c<'static, mode::Blocking>;
pub type SensorI2c = embassy_stm32::i2c::I2c<'static, mode::Async>;
#[allow(dead_code)]
pub type SensorDevice = I2cDevice<'static, NoopRawMutex, SensorI2c>;
pub type OutputPin = Output<'static>;
pub type Crc = embassy_stm32::crc::Crc<'static>;

pub type Spi = spi::Spi<'static, embassy_stm32::mode::Async>;

use w25q32jv::W25q32jv;

pub type SpiDevice =
    embedded_hal_bus::spi::ExclusiveDevice<Spi, OutputPin, embedded_hal_bus::spi::NoDelay>;
pub type Flash = W25q32jv<SpiDevice, OutputPin, OutputPin>;
pub type Leds = [Output<'static>; 5];

#[derive(Clone)]
pub struct BufferedCanErrorSender {
    txbuf: can::BufferedCanSender,
}

impl BufferedCanErrorSender {
    pub fn new(txbuf: can::BufferedCanSender) -> Self {
        Self { txbuf }
    }
}

impl j1939::error::SendError for BufferedCanErrorSender {
    fn send(&mut self, error: &j1939::error::Error) {
        match can::Frame::new_extended(0xEEE, &error.ebytes) {
            Ok(frame) => match self.txbuf.try_write(frame) {
                Ok(_) => {}
                Err(_) => {}
            },

            Err(_) => {}
        }
    }
}

#[allow(dead_code)]
pub struct DisplayConnector {
    pub spi: Spi,
    pub cs: Output<'static>,
    pub dc: Output<'static>,
    pub rst: Output<'static>,
    pub backlight: Output<'static>,
}

pub struct Bsp(
    pub u32,                      // Device ID
    pub CanConfigurator<'static>, // CAN
    pub Output<'static>,          // Cansleep
    pub ExtiPin,                  // Ignition
    pub Leds,
    pub NvI2c,
    pub SensorI2c,
    pub MonitorAlertPins,
    pub InputPins,
    pub Flash,
    pub Crc,
    pub DisplayConnector,
);

#[allow(dead_code)]
#[cfg(feature = "js1")]
pub mod consts {
    pub const XTAL_FREQUENCY: u32 = 16_000_000;
    pub const INA226_ADDRS: [u8; 4] = [0b1000000, 0b1000001, 0b1000010, 0b1000011];
}
#[cfg(not(feature = "js1"))]
pub mod consts {
    pub const XTAL_FREQUENCY: u32 = 8_000_000;
    pub const INA226_ADDRS: [u8; 4] = [0b1000000, 0b1000001, 0b1000100, 0b1000101];
}
impl Bsp {
    pub fn new() -> Self {
        use embassy_stm32::time::Hertz;

        let mut config = embassy_stm32::Config::default();
        //defmt::println!("Clock {}", config.rcc.hse);

        {
            use embassy_stm32::rcc::*;
            config.rcc.hse = Some(Hse {
                freq: Hertz(consts::XTAL_FREQUENCY),
                mode: HseMode::Oscillator,
            });
            let use_pll = false;
            if use_pll {
                //config.rcc.mux.fdcansel = mux::Fdcansel::HSE;
                config.rcc.pll = Some(Pll {
                    source: PllSource::HSE,
                    prediv: PllPreDiv::DIV2,
                    mul: PllMul::MUL85,
                    divp: None,
                    divq: Some(PllQDiv::DIV8), // 42.5 Mhz for fdcan.
                    divr: Some(PllRDiv::DIV2), // Main system clock at 170 MHz
                });
                config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
                config.rcc.sys = Sysclk::PLL1_R;
            } else {
                config.rcc.mux.fdcansel = mux::Fdcansel::HSE;
                config.rcc.sys = Sysclk::HSE;
            }
        }
        let p = embassy_stm32::init(config);

        let device_id = j1939::name::identiy_from_bytes(embassy_stm32::uid::uid());
        //defmt::println!("DID {:?}->{:x}", embassy_stm32::uid::uid(), device_id);
        let can_iface = can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, CanIrqs);
        let mut cansleep = Output::new(p.PA10, Level::Low, Speed::Low);
        cansleep.set_low();

        //let mut rtc = embassy_stm32::rtc::Rtc::new(p.RTC, embassy_stm32::rtc::RtcConfig::default());
        //rtc.now();

        // Setup Heartbeat LED
        #[cfg(feature = "js1")]
        let leds = [
            Output::new(p.PB8, Level::High, Speed::Low),
            Output::new(p.PB9, Level::High, Speed::Low),
            Output::new(p.PB1, Level::High, Speed::Low),
            Output::new(p.PC11, Level::High, Speed::Low),
            Output::new(p.PC10, Level::High, Speed::Low),
        ];
        #[cfg(not(feature = "js1"))]
        let led = Output::new(p.PC6, Level::High, Speed::Low);

        // Setup I2C Bus manager
        let nvstore_i2c = embassy_stm32::i2c::I2c::new_blocking(
            p.I2C2,
            p.PA9, // SCL
            p.PA8, // SDA
            //I2c2Irqs,
            //embassy_stm32::dma::NoDma,
            //embassy_stm32::dma::NoDma,
            Hertz(100_000),
            Default::default(),
        );
        use embassy_stm32::i2c::*;
        // Setup I2C Bus manager
        let sensors_i2c = I2c::new(
            p.I2C1,
            p.PA15, // SCL
            p.PB7,  // SDA
            I2c1Irqs,
            p.DMA1_CH1,
            p.DMA1_CH2,
            Hertz(400_000),
            Default::default(),
        );

        let crc_config = embassy_stm32::crc::Config::new(
            embassy_stm32::crc::InputReverseConfig::None,
            false,
            embassy_stm32::crc::PolySize::Width32,
            0xFFFFFFFF,
            0x04C11DB7,
        )
        .unwrap();
        let crc = embassy_stm32::crc::Crc::new(p.CRC, crc_config);

        #[cfg(feature = "js1")]
        let sensor_alerts = [
            ExtiInput::new(p.PB5, p.EXTI5, Pull::Up),
            ExtiInput::new(p.PB4, p.EXTI4, Pull::Up),
            ExtiInput::new(p.PA1, p.EXTI1, Pull::Up),
            ExtiInput::new(p.PA0, p.EXTI0, Pull::Up),
        ];
        #[cfg(not(feature = "js1"))]
        let sensor_alerts = [
            ExtiInput::new(p.PB5, p.EXTI5, Pull::Up),
            ExtiInput::new(p.PB4, p.EXTI4, Pull::Up),
            ExtiInput::new(p.PA0, p.EXTI0, Pull::Up),
            ExtiInput::new(p.PA1, p.EXTI1, Pull::Up),
        ];

        let encoder_inputs = [
            ExtiInput::new(p.PC13, p.EXTI13, Pull::Up),
            ExtiInput::new(p.PB3, p.EXTI3, Pull::Up),
            ExtiInput::new(p.PB11, p.EXTI11, Pull::Up),
        ];

        let ignition_pin = ExtiInput::new(p.PB6, p.EXTI6, Pull::Down);

        let flash = {
            let cs = Output::new(p.PB12, Level::High, Speed::Low);
            let wp = Output::new(p.PC6, Level::High, Speed::Low);
            let reset = Output::new(p.PB2, Level::High, Speed::Low);
            let sck = p.PB13;
            let miso = p.PB14;
            let mosi = p.PB15;
            let mut config = spi::Config::default();
            config.frequency = Hertz(16_000_000);

            let spi = embassy_stm32::spi::Spi::new(
                p.SPI2, sck, mosi, miso, p.DMA2_CH1, p.DMA2_CH2, config,
            );
            let spi: SpiDevice = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs);

            // Create the flash driver instance
            W25q32jv::new(spi, reset, wp).unwrap()
        };

        let display_connector = {
            let sck = p.PA5;
            let miso = p.PA6;
            let mosi = p.PA7;
            let mut config = spi::Config::default();
            config.frequency = Hertz(16_000_000);

            let spi = embassy_stm32::spi::Spi::new(
                p.SPI1, sck, mosi, miso, p.DMA2_CH3, p.DMA2_CH4, config,
            );

            let dc = Output::new(p.PA2, Level::High, Speed::Low);
            let cs = Output::new(p.PA3, Level::High, Speed::Low);
            let rst = Output::new(p.PA4, Level::High, Speed::Low);
            let backlight = Output::new(p.PB0, Level::High, Speed::Low);

            DisplayConnector {
                spi,
                cs,
                dc,
                rst,
                backlight,
            }
        };

        Bsp(
            device_id,
            can_iface,
            cansleep,
            ignition_pin,
            leds,
            nvstore_i2c,
            sensors_i2c,
            sensor_alerts,
            encoder_inputs,
            flash,
            crc,
            display_connector,
        )
    }
}
