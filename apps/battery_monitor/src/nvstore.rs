//use sequential_storage::cache::NoCache;
use crate::error::*;
use core::ops::Range;
#[cfg(feature = "power_sensors")]
use embedded_storage_async::nor_flash::NorFlash;
use j1939::error::mkerr_generic;
use w25q32jv::W25q32jv;

#[allow(unused_variables)]

type SpiDevice = embedded_hal_bus::spi::ExclusiveDevice<
    crate::bsp::Spi,
    crate::bsp::OutputPin,
    embedded_hal_bus::spi::NoDelay,
>;
type Flash = W25q32jv<SpiDevice, crate::bsp::OutputPin, crate::bsp::OutputPin>;

type PinError = core::convert::Infallible;
type SpiError = embedded_hal_bus::spi::DeviceError<embassy_stm32::spi::Error, PinError>;
//type SpiError = SpiDevice::Error;
//type PinError = crate::bsp::OutputPin::Error;
type FlashError = w25q32jv::Error<SpiError, PinError>;
type StoreError = sequential_storage::Error<FlashError>;

const FILE_CODE: u8 = 0x04;

const MEGABYTE: u32 = 1024 * 1024;
const CONF_FLASH_RANGE: Range<u32> = 0 * MEGABYTE..1 * MEGABYTE;
const INDEX_FLASH_RANGE: Range<u32> = 1 * MEGABYTE..2 * MEGABYTE;
const DATA_FLASH_RANGE: Range<u32> = 4 * MEGABYTE..16 * MEGABYTE;

use sequential_storage::cache::NoCache;

#[allow(async_fn_in_trait)]
pub trait PdStore {
    async fn store_pd(
        &mut self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
        value: i32,
    ) -> Result<(), Error>;
    async fn get_pd(
        &mut self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
    ) -> Result<Option<i32>, Error>;
}

pub mod power_sensors {
    use binary_layout::prelude::*;
    pub struct Header {
        pub time: i64, // Time since Epoch
    }

    impl Header {
        pub fn new() -> Self {
            Header { time: 0 }
        }
    }

    define_layout!(record_header, BigEndian, {
        time: i64,    // Time since Epoch
    });

    define_layout!(record_tail, BigEndian, {
        crc: u32,     // CRC32
    });

    define_layout!(record_one_monitor, BigEndian, {
        total_charge_ua_ms: i64,    // Amp-seconds
        charge_in_ua_ms: u64,       // Amp-seconds
        charge_out_ua_ms: u64,      // Amp-seconds
    });

    pub const HEADER_SIZE: usize = record_header::SIZE.unwrap();
    pub const OBSERVATION_SIZE: usize = record_one_monitor::SIZE.unwrap();
    pub const TAIL_SIZE: usize = record_tail::SIZE.unwrap();
    pub const NUM_MONITORS: usize = 4;
    pub const RECORD_SIZE: usize = HEADER_SIZE + TAIL_SIZE + (NUM_MONITORS * OBSERVATION_SIZE);
    pub const FULL_RECORD_SIZE: u32 = 256;
}
const KEY_WRITE: u8 = 0;
const KEY_READ: u8 = 1;

pub const ELEM_MIN_POWER_MON_CHAN0: j1939::process_data::ElementNumber = 100;

pub struct NvStore {
    flash: Flash,
    #[cfg(feature = "power_sensors")]
    crc: crate::bsp::Crc,
    read: u32,
    write: u32,
    num_saves_done: usize,
}

pub fn pd_as_key(elem: j1939::process_data::ElementNumber, ddi: j1939::process_data::Ddi) -> u32 {
    (elem as u32) << 16 | (ddi as u32)
}

#[cfg(feature = "power_sensors")]
fn mon_index_move(index_in: u32, records: i32) -> u32 {
    let mut index = index_in;
    if records >= 0 {
        index += power_sensors::FULL_RECORD_SIZE * records as u32;
        if index >= DATA_FLASH_RANGE.end {
            index -= DATA_FLASH_RANGE.end - DATA_FLASH_RANGE.start;
        }
    } else {
        let ammount = power_sensors::FULL_RECORD_SIZE * (-records) as u32;
        //if DATA_FLASH_RANGE.start < index - ammount{
        if index < DATA_FLASH_RANGE.start + ammount {
            index += index + (DATA_FLASH_RANGE.end - DATA_FLASH_RANGE.start) - ammount;
        } else {
            index -= ammount;
        }
    }
    //defmt::println!("NV in={:x} i={:?} out={:x}", index_in, records, index);
    index
}

impl NvStore {
    pub fn new(flash: Flash, #[cfg(feature = "power_sensors")] crc: crate::bsp::Crc) -> Self {
        NvStore {
            flash,
            #[cfg(feature = "power_sensors")]
            crc,
            read: 0,
            write: 0,
            num_saves_done: 0,
        }
    }

    pub async fn erase_all_pd(&mut self) -> Result<(), StoreError> {
        sequential_storage::erase_all(&mut self.flash, CONF_FLASH_RANGE).await
    }

    pub async fn store_config(&mut self, key: u32, value: i32) -> Result<(), Error> {
        let mut data_buffer = [0; 32];
        sequential_storage::map::store_item(
            &mut self.flash,
            CONF_FLASH_RANGE.clone(),
            &mut NoCache::new(),
            &mut data_buffer,
            &key,
            &value,
        )
        .await
        .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::FlashWrite as u8, line!(), e))
    }

    pub async fn get_config(&mut self, key: u32) -> Result<Option<i32>, Error> {
        let mut data_buffer = [0; 32];
        sequential_storage::map::fetch_item::<u32, i32, _>(
            &mut self.flash,
            CONF_FLASH_RANGE.clone(),
            &mut NoCache::new(),
            &mut data_buffer,
            &key,
        )
        .await
        .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::FlashRead as u8, line!(), e))
    }

    pub async fn store_pd(
        &mut self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
        value: i32,
    ) -> Result<(), Error> {
        //defmt::println!("Storing elem={:x} ddi={:x} value={}", elem, ddi, value);
        self.store_config(pd_as_key(elem, ddi), value).await
    }

    pub async fn get_pd(
        &mut self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
    ) -> Result<Option<i32>, Error> {
        self.get_config(pd_as_key(elem, ddi)).await
    }

    pub async fn init_pd(
        &mut self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
        default_value: i32,
        errstate: &mut Result<(), Error>,
    ) -> i32 {
        match self
            .get_pd(elem, ddi)
            .await
            .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::PdMissing as u8, line!(), e))
        {
            Ok(Some(value)) => value,
            Ok(None) => {
                //defmt::println!("Can't find config value elem={:x} ddi={:x}", elem, ddi);
                default_value
            }

            Err(e) => {
                *errstate = Err(e);
                //defmt::println!("Error finding elem={:x} ddi={:x}", elem, ddi);
                default_value
            }
        }
    }

    pub async fn store_index(&mut self, key: u8, value: u32) -> Result<(), Error> {
        let mut data_buffer = [0; 32];
        sequential_storage::map::store_item(
            &mut self.flash,
            INDEX_FLASH_RANGE.clone(),
            &mut NoCache::new(),
            &mut data_buffer,
            &key,
            &value,
        )
        .await
        .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::IndexMissing as u8, line!(), e))
    }

    pub async fn get_index(&mut self, key: u8) -> Result<Option<u32>, Error> {
        let mut data_buffer = [0; 32];
        sequential_storage::map::fetch_item::<u8, u32, _>(
            &mut self.flash,
            INDEX_FLASH_RANGE.clone(),
            &mut NoCache::new(),
            &mut data_buffer,
            &key,
        )
        .await
        .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::IndexMissing as u8, line!(), e))
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        let write = self.get_index(KEY_WRITE).await?;
        let read = self.get_index(KEY_READ).await?;
        match (write, read) {
            (Some(write), Some(read))
                if read >= DATA_FLASH_RANGE.start as u32
                    && read < DATA_FLASH_RANGE.end as u32
                    && write >= DATA_FLASH_RANGE.start as u32
                    && write < DATA_FLASH_RANGE.end as u32 =>
            {
                self.write = write;
                self.read = read;
                self.store_index(KEY_WRITE, self.write).await?;
                self.store_index(KEY_READ, self.read).await?;
            }
            _ => {
                self.write = DATA_FLASH_RANGE.start;
                self.read = DATA_FLASH_RANGE.start;
                self.store_index(KEY_WRITE, self.write).await?;
                self.store_index(KEY_READ, self.read).await?;
            }
        }
        //============= Need to store all 3 values on occation.
        Ok(())
    }
    pub async fn store_indexes(&mut self, write: u32, read: u32) -> Result<(), Error> {
        let refresh = (self.num_saves_done & 0xF) == 0xF;
        if refresh || write != self.write {
            self.write = write;
            self.store_index(KEY_WRITE, write).await?;
        }
        if refresh || read != self.read {
            self.read = read;
            self.store_index(KEY_READ, read).await?;
        }
        return Ok(());
    }

    /*pub async fn load_monitor_settings(&mut self) -> Result<(), Error> {
        let mut unlocked = self.data_store.lock().await;

        let mut ret = Ok(());
        for i in 0..NUM_MONITORS {
            if i >= unlocked.monitors.len() {
                continue;
            }
            let mon = unlocked.monitors.monitor_as_mut(i);
            mon.set_shunt_resistance_u(
                self.init_pd(
                    ELEM_MIN_POWER_MON_CHAN0 + i as u16,
                    j1939::process_data::DDI_PROP_SHUNT_RESISTANCE,
                    1_500,
                    &mut ret,
                )
                .await,
            );
            mon.clear_shunt_resistance_changed();
        }
        ret
    }*/

    /*pub async fn store_dirty_settings(&mut self) -> Result<(), Error> {
        let mut unlocked = self.data_store.lock().await;

        for i in 0..NUM_MONITORS {
            let mon = unlocked.monitors.monitor_as_mut(i);
            if !mon.shunt_resistance_changed() {
                continue;
            }
            self.store_pd(
                ELEM_MIN_POWER_MON_CHAN0 + i as u16,
                j1939::process_data::DDI_PROP_SHUNT_RESISTANCE,
                mon.shunt_resistance_u(),
            )
            .await?;
            mon.clear_shunt_resistance_changed();
        }

        Ok(())
    }*/

    #[cfg(feature = "power_sensors")]
    pub async fn pack_monitor_observation(
        ds: &crate::application::Data,
        data: &mut [u8; power_sensors::FULL_RECORD_SIZE as usize],
        //_time: crate::types::TimeInstant,
        header: &power_sensors::Header,
    ) -> Result<(), Error> {
        //let unlocked = self.data_store.lock().await;

        {
            let mut state =
                power_sensors::record_header::View::new(&mut data[0..power_sensors::HEADER_SIZE]);
            state.time_mut().write(header.time);
        }
        for i in 0..power_sensors::NUM_MONITORS {
            let mut state = power_sensors::record_one_monitor::View::new(
                &mut data[power_sensors::HEADER_SIZE + i * power_sensors::OBSERVATION_SIZE..],
            );
            let mon = ds.monitors.monitor(i);

            state
                .total_charge_ua_ms_mut()
                .write(mon.total_charge_ua_ms());
            state.charge_in_ua_ms_mut().write(mon.charge_in_ua_ms());
            state.charge_out_ua_ms_mut().write(mon.charge_out_ua_ms());
        }

        Ok(())
    }

    #[cfg(feature = "power_sensors")]
    pub async fn write_monitor_observation(
        &mut self,
        data: &mut [u8; power_sensors::FULL_RECORD_SIZE as usize],
        //_time: crate::types::TimeInstant,
        //header: &Header,
    ) -> Result<(), Error> {
        let mut write = self.write;
        let mut read = self.read;
        let crc;
        {
            {
                self.crc.reset();
                crc = self.crc.feed_bytes(
                    &data[0..power_sensors::FULL_RECORD_SIZE as usize - power_sensors::TAIL_SIZE],
                );
                let mut state = power_sensors::record_tail::View::new(
                    &mut data[power_sensors::FULL_RECORD_SIZE as usize - power_sensors::TAIL_SIZE
                        ..power_sensors::FULL_RECORD_SIZE as usize],
                );
                state.crc_mut().write(crc);
            }
        }

        // Enforce always writing/erasing to page boundaries
        assert!(Flash::ERASE_SIZE as u32 % power_sensors::FULL_RECORD_SIZE == 0);
        assert!(power_sensors::RECORD_SIZE as u32 <= power_sensors::FULL_RECORD_SIZE);
        assert!(write >= DATA_FLASH_RANGE.start);
        assert!(write < DATA_FLASH_RANGE.end);
        if write % Flash::ERASE_SIZE as u32 == 0 {
            // Start of sector, need to erase
            let erase_end = write + Flash::ERASE_SIZE as u32;
            self.flash
                .erase(write, erase_end)
                .await
                .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::FlashErase as u8, line!(), e))?;
            if read >= write && read < erase_end {
                read = erase_end;
            }
            if read >= DATA_FLASH_RANGE.end {
                read = DATA_FLASH_RANGE.start;
            }
        }

        let ret = self.flash.write_async(write, data).await;

        // Bump indexes
        write = mon_index_move(write, 1);
        if read == write {
            read = mon_index_move(read, 1);
        }

        //defmt::println!("NV {:x} {:x}", write, crc);
        /*defmt::println!(
            "NV >>>>>> r={:x}->{:x} w={:x}->{:x} mw={:x} me={:x}",
            self.read,
            read,
            self.write,
            write,
            Flash::ERASE_SIZE,
            Flash::WRITE_SIZE,
        );*/
        self.num_saves_done += 1;
        self.store_indexes(write, read).await?;

        // Process the error down here. In case we get a ReadbackFail, we still want to bump the pointers.
        // Readback fail can happen on first chunk if
        ret.map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::FlashWrite as u8, line!(), e))?;

        Ok(())
    }

    #[cfg(feature = "power_sensors")]
    pub async fn store_monitor_observation(
        &mut self,
        data_store: &'static crate::application::DataStore,
        _time: crate::types::TimeInstant,
        header: &power_sensors::Header,
    ) -> Result<(), Error> {
        let mut data = [0x00; power_sensors::FULL_RECORD_SIZE as usize];

        {
            let unlocked = data_store.lock().await;

            Self::pack_monitor_observation(&unlocked, &mut data, header).await?;
        }
        self.write_monitor_observation(&mut data).await?;
        Ok(())
    }

    #[cfg(feature = "power_sensors")]
    pub async fn load_last_monitor_observation(
        &mut self,
        data_store: &'static crate::application::DataStore,
        index: i32,
        //time: crate::types::TimeInstant,
        header: &mut power_sensors::Header,
    ) -> Result<bool, Error> {
        let addr = match index {
            index if index < 0 => mon_index_move(self.write, index),
            index => mon_index_move(self.read, index),
        };
        let mut data = [0x00; power_sensors::FULL_RECORD_SIZE as usize];

        self.flash
            .read_async(addr, &mut data)
            .await
            .map_err(|e| mkerr_generic(FILE_CODE, ErrorCode::FlashRead as u8, line!(), e))?;

        {
            self.crc.reset();
            let calc_crc = self.crc.feed_bytes(
                &data[0..power_sensors::FULL_RECORD_SIZE as usize - power_sensors::TAIL_SIZE],
            );
            let state = power_sensors::record_tail::View::new(
                &mut data[power_sensors::FULL_RECORD_SIZE as usize - power_sensors::TAIL_SIZE
                    ..power_sensors::FULL_RECORD_SIZE as usize],
            );
            let load_crc = state.crc().read();
            defmt::println!("NV CRC {:x} vs {:x}", calc_crc, load_crc);
            // FIXME: Implement error handling for CRC mismatch
        }

        return self.decode_mon_data(data_store, header, &data).await;
    }

    #[cfg(feature = "power_sensors")]
    pub async fn decode_mon_data(
        &mut self,
        data_store: &'static crate::application::DataStore,
        //time: crate::types::TimeInstant,
        header: &mut power_sensors::Header,
        data: &[u8],
    ) -> Result<bool, Error> {
        if data.len() < power_sensors::RECORD_SIZE {
            return Ok(false);
        }
        let mut unlocked = data_store.lock().await;

        {
            let state =
                power_sensors::record_header::View::new(&data[0..power_sensors::HEADER_SIZE]);
            header.time = state.time().read();
        }

        for i in 0..power_sensors::NUM_MONITORS {
            let state = power_sensors::record_one_monitor::View::new(
                &data[power_sensors::HEADER_SIZE + i * power_sensors::OBSERVATION_SIZE..],
            );
            let mon = unlocked.monitors.monitor_as_mut(i);
            mon.set_total_charge_ua_ms(state.total_charge_ua_ms().read());
            mon.set_charge_in_ua_ms(state.charge_in_ua_ms().read());
            mon.set_charge_out_ua_ms(state.charge_out_ua_ms().read());
        }

        Ok(true)
    }

    pub fn done(self: Self) -> Flash {
        self.flash
    }
}

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
pub struct SharedNvStore {
    //pub nv: Mutex<CriticalSectionRawMutex, crate::nvstore::NvStore>,
    pub nv: Mutex<CriticalSectionRawMutex, Option<NvStore>>,
}

impl SharedNvStore {
    //pub fn new(nv: crate::nvstore::NvStore) -> Self {
    //    NvStore { nv: Mutex::new(nv) }
    //}
    pub const fn new() -> Self {
        SharedNvStore {
            nv: Mutex::new(None),
        }
    }

    pub async fn get_pd(
        &mut self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
    ) -> Result<Option<i32>, Error> {
        let mut unlocked = self.nv.lock().await;
        let nvs = unlocked.as_mut().ok_or(crate::error::mkerr(
            FILE_CODE,
            ErrorCode::NoDevice,
            line!(),
        ))?;
        nvs.get_pd(elem, ddi).await
    }

    pub async fn get_pd_with_default(
        &self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
        default: i32,
        store_default: bool,
    ) -> Result<i32, Error> {
        defmt::println!("Load PD {} {}", elem, ddi);
        let mut unlocked = self.nv.lock().await;
        let nvs = unlocked.as_mut().ok_or(crate::error::mkerr(
            FILE_CODE,
            ErrorCode::NoDevice,
            line!(),
        ))?;
        match nvs.get_pd(elem, ddi).await? {
            Some(value) => {
                defmt::println!("Got shunt {}", value);
                Ok(value)
            }
            None => {
                defmt::println!("Using default value {}", default);
                if store_default {
                    nvs.store_pd(elem, ddi, default).await?;
                }
                Ok(default)
            }
        }
    }

    pub async fn store_pd(
        &self,
        elem: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
        value: i32,
    ) -> Result<(), Error> {
        let mut unlocked = self.nv.lock().await;
        let nvs = unlocked.as_mut().ok_or(crate::error::mkerr(
            FILE_CODE,
            ErrorCode::NoDevice,
            line!(),
        ))?;
        nvs.store_pd(elem, ddi, value).await
    }

    #[cfg(feature = "power_sensors")]
    pub async fn store_monitor_observation(
        &self,
        data_store: &crate::application::DataStore,
        _time: crate::types::TimeInstant,
        header: &power_sensors::Header,
    ) -> Result<(), Error> {
        let mut data = [0x00; power_sensors::FULL_RECORD_SIZE as usize];

        {
            let unlocked = data_store.lock().await;

            NvStore::pack_monitor_observation(&unlocked, &mut data, header).await?;
        }
        {
            let mut unlocked = self.nv.lock().await;
            let nvs = unlocked.as_mut().ok_or(crate::error::mkerr(
                FILE_CODE,
                ErrorCode::NoDevice,
                line!(),
            ))?;
            nvs.write_monitor_observation(&mut data).await?;
        }
        defmt::println!("Store observation Done");
        Ok(())
    }
}
