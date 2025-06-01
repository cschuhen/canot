use j1939::process_data;

//use rtt_target::{defmt::println};
use crate::{mkerr, nvstore::ELEM_MIN_POWER_MON_CHAN0};

const FILE_CODE: u8 = 0x03;

pub const MAX_MONITORS: usize = 4;
const SHUNT_VOLTAGE_LSB_PV: i64 = 2_500_000; // 2.5 μV = 2,500,000pV.
const BUS_VOLTAGE_LSB_UV: i32 = 1_250; // 1.25 mV = 1_250uV.

pub async fn handle_settable_pd_value(
    nvstore: &'static crate::nvstore::SharedNvStore,
    pd: &process_data::ProcessData,
    value: &mut i32,
) -> process_data::HandlePdResult {
    let ret = process_data::handle_settable_pd_value(pd, value)?;
    if ret.changed() {
        nvstore.store_pd(pd.element_number, pd.ddi, *value).await?;
    }
    Ok(ret)
}

#[derive(Clone)]
pub struct Observation {
    pub monitor: u8,
    pub bus_voltage_raw: u16,
    pub shunt_voltage_raw: i16,
}

impl Observation {
    pub fn new(monitor: u8, bus_voltage_raw: u16, shunt_voltage_raw: i16) -> Self {
        //defmt::println!("Read {} {} {}", monitor, bus_voltage_raw, shunt_voltage_raw);
        Observation {
            monitor,
            bus_voltage_raw,
            shunt_voltage_raw,
        }
    }
}

pub struct Monitor {
    last_save: crate::types::TimeInstant,
    total_charge_ua_ms: i64,   // micro-Amp-milli-seconds
    charge_in_ua_ms: u64,      // micro-Amp-milli-seconds
    charge_out_ua_ms: u64,     // micro-Amp-milli-seconds
    shunt_current_ua: i32,     // Micro-amp
    bus_voltage_uv: i32,       // micro-volts
    shunt_resistance_u: i32,   // micro-ohms
    shunt_voltage_offset: i32, // ADC counts (2.5 μV)
    shunt_resistance_changed: bool,
    last_time: crate::types::TimeInstant,
}

impl Monitor {
    pub fn new(now: crate::types::TimeInstant) -> Self {
        Monitor {
            last_save: now,
            total_charge_ua_ms: 0,
            charge_in_ua_ms: 0,
            charge_out_ua_ms: 0,
            shunt_current_ua: 0,
            bus_voltage_uv: 0,
            shunt_resistance_u: 1_000,
            shunt_voltage_offset: 0,
            shunt_resistance_changed: false,
            last_time: now,
        }
    }

    pub fn last_save_time(&self) -> crate::types::TimeInstant {
        self.last_save
    }
    pub fn set_last_save_time(&mut self, t: crate::types::TimeInstant) {
        self.last_save = t;
    }

    pub fn total_charge_ua_ms(&self) -> i64 {
        self.total_charge_ua_ms
    }
    pub fn total_charge_uah(&self) -> i32 {
        (self.total_charge_ua_ms / 3_600_000) as i32
    }
    pub fn charge_in_ua_ms(&self) -> u64 {
        self.charge_in_ua_ms
    }
    pub fn charge_out_ua_ms(&self) -> u64 {
        self.charge_out_ua_ms
    }
    pub fn shunt_resistance_u(&self) -> i32 {
        self.shunt_resistance_u
    }
    pub fn bus_voltage_uv(&self) -> i32 {
        self.bus_voltage_uv
    }
    pub fn current_ua(&self) -> i32 {
        self.shunt_current_ua
    }
    pub fn shunt_resistance_changed(&self) -> bool {
        self.shunt_resistance_changed
    }
    pub fn clear_shunt_resistance_changed(&mut self) {
        self.shunt_resistance_changed = false;
    }

    pub fn set_total_charge_ua_ms(&mut self, val: i64) {
        self.total_charge_ua_ms = val;
    }
    pub fn set_total_charge_a_h(&mut self, charge_a_h: f64) {
        let charge_a_ms = charge_a_h * 3600.0 * 1000.0;
        let charge_ua_ms = charge_a_ms * 1_000_000.0;
        self.set_total_charge_ua_ms(charge_ua_ms as i64);
    }
    pub fn set_charge_in_ua_ms(&mut self, val: u64) {
        self.charge_in_ua_ms = val;
    }
    pub fn set_charge_out_ua_ms(&mut self, val: u64) {
        self.charge_out_ua_ms = val;
    }
    pub fn set_shunt_resistance_u(&mut self, val: i32) {
        if self.shunt_resistance_u != val {
            self.shunt_resistance_u = val;
            self.shunt_resistance_changed = true;
        }
    }

    pub fn handle_observation(
        &mut self,
        time: crate::types::TimeInstant,
        obs: &Observation,
    ) -> Result<bool, crate::Error> {
        let tdelta_ms = (time - self.last_time).as_millis() as i64;
        // mon.last_save_time()).to_secs() < 10 {
        //let tdelta_ms = time_ms.wrapping_sub(self.last_time_ms) as i64;

        self.bus_voltage_uv = (obs.bus_voltage_raw as i32) * BUS_VOLTAGE_LSB_UV;
        let shunt_voltage_raw = obs.shunt_voltage_raw as i32 + self.shunt_voltage_offset;
        let shunt_voltage_pv = (shunt_voltage_raw as i64) * SHUNT_VOLTAGE_LSB_PV;

        self.shunt_current_ua = (shunt_voltage_pv / self.shunt_resistance_u as i64) as i32;

        let shunt_charge_ua_ms =
            (shunt_voltage_pv as i64 * tdelta_ms) / (self.shunt_resistance_u as i64);

        //let charge = shunt_current_ua * tdelta_ms;
        self.total_charge_ua_ms += shunt_charge_ua_ms;

        if shunt_charge_ua_ms > 0 {
            self.charge_in_ua_ms = match self.charge_in_ua_ms.checked_add(shunt_charge_ua_ms as u64)
            {
                None => {
                    if self.charge_in_ua_ms == 0xFFFFFFFFFFFFFFFF {
                        defmt::println!("BAD self.charge_in_ua_ms All F's");
                        0
                    } else {
                        defmt::println!(
                            "BAD self.charge_in_ua_ms {} {}",
                            self.charge_in_ua_ms,
                            shunt_charge_ua_ms
                        );
                        self.charge_in_ua_ms
                    }
                }
                Some(sum) => sum,
            }
        } else if shunt_charge_ua_ms < 0 {
            self.charge_out_ua_ms =
                match self.charge_in_ua_ms.checked_add(-shunt_charge_ua_ms as u64) {
                    None => {
                        if self.charge_out_ua_ms == 0xFFFFFFFFFFFFFFFF {
                            defmt::println!("BAD self.charge_out_ua_ms All F's");
                            0
                        } else {
                            defmt::println!(
                                "BAD self.charge_out_ua_ms {} {}",
                                self.charge_in_ua_ms,
                                shunt_charge_ua_ms
                            );
                            self.charge_out_ua_ms
                        }
                    }
                    Some(sum) => sum,
                }
        }
        /*if obs.monitor < 10 {
            defmt::println!(
                "  INT   @{} Mon={} Bus={}mv Shunt={:04}uv {:08}uA charge={:08}uAms,{}uAh total={}uAh in={:X}uams out={:X}uams",
                //time_ms,
                tdelta_ms,
                obs.monitor,
                self.bus_voltage_uv / 1000,
                shunt_voltage_pv / 1000000, // pv to uv
                self.shunt_current_ua,
                shunt_charge_ua_ms,
                shunt_charge_ua_ms / 3_600_000,
                self.total_charge_ua_ms / 3_600_000,
                self.charge_in_ua_ms,
                self.charge_out_ua_ms,
            );
        }*/
        self.last_time = time;
        Ok(true)
    }

    pub async fn handle_process_data(
        &mut self,
        nvstore: &'static crate::nvstore::SharedNvStore,
        pd: &process_data::ProcessData,
    ) -> process_data::HandlePdResult {
        match pd.ddi {
            j1939::process_data::DDI_PROP_SHUNT_RESISTANCE => {
                handle_settable_pd_value(nvstore, pd, &mut self.shunt_resistance_u).await
            }
            j1939::process_data::DDI_PROP_SHUNT_OFFSET => {
                handle_settable_pd_value(nvstore, pd, &mut self.shunt_voltage_offset).await
            }
            _ => Ok(process_data::PdReturn::new_not_handled()),
        }
    }
}

type MonitorVec = heapless::Vec<Monitor, MAX_MONITORS>;
pub struct Monitors {
    monitors: MonitorVec,
    nvstore: &'static crate::nvstore::SharedNvStore,
}

impl Monitors {
    pub const fn new(nvstore: &'static crate::nvstore::SharedNvStore) -> Self {
        let monitors = MonitorVec::new();

        Monitors { monitors, nvstore }
    }

    pub fn monitor(&self, idx: usize) -> &Monitor {
        &self.monitors[idx]
    }

    pub fn monitor_as_mut(&mut self, idx: usize) -> &mut Monitor {
        &mut self.monitors[idx]
    }

    pub fn init_data(&mut self, idx: usize, resistance_u: i32, charge_a_h: f64) {
        if idx >= self.monitors.len() {
            return;
        }
        self.monitors[idx].set_shunt_resistance_u(resistance_u);
        self.monitors[idx].set_total_charge_a_h(charge_a_h);
        self.monitors[idx].set_charge_in_ua_ms(0);
        self.monitors[idx].set_charge_out_ua_ms(0);
    }

    pub async fn load_config_from_nvstore(&mut self) -> Result<(), crate::Error> {
        for idx in 0..self.monitors.len() {
            let monitor = &mut self.monitors[idx];
            monitor.shunt_resistance_u = self
                .nvstore
                .get_pd_with_default(
                    ELEM_MIN_POWER_MON_CHAN0 + idx as u16,
                    j1939::process_data::DDI_PROP_SHUNT_RESISTANCE,
                    1_500,
                    false,
                )
                .await?;
            monitor.shunt_voltage_offset = self
                .nvstore
                .get_pd_with_default(
                    ELEM_MIN_POWER_MON_CHAN0 + idx as u16,
                    j1939::process_data::DDI_PROP_SHUNT_OFFSET,
                    0,
                    false,
                )
                .await?;
        }

        Ok(())
    }

    pub async fn handle_process_data(
        &mut self,
        pd: &process_data::ProcessData,
    ) -> process_data::HandlePdResult {
        if pd.element_number < ELEM_MIN_POWER_MON_CHAN0
            || pd.element_number >= (ELEM_MIN_POWER_MON_CHAN0 + self.monitors.len() as u16)
        {
            return Ok(process_data::PdReturn::new_not_handled());
        }
        self.monitors[(pd.element_number - ELEM_MIN_POWER_MON_CHAN0) as usize]
            .handle_process_data(self.nvstore, pd)
            .await
    }

    pub fn len(&self) -> usize {
        self.monitors.len()
    }

    pub async fn setup(
        &mut self,
        count: usize,
        time: crate::types::TimeInstant,
    ) -> Result<(), crate::Error> {
        self.monitors = MonitorVec::new();
        for _ in 0..count {
            if self.monitors.push(Monitor::new(time)).is_err() {
                return Err(mkerr(FILE_CODE, crate::ErrorCode::NoSpace, line!()));
            }
        }
        self.load_config_from_nvstore().await?;
        Ok(())
    }

    pub fn handle_observation(
        &mut self,
        time: crate::types::TimeInstant,
        obs: &Observation,
    ) -> Result<bool, crate::Error> {
        self.monitors[obs.monitor as usize].handle_observation(time, obs)
    }
}
