use crate::error::mkerr;
use crate::error::Error;
use crate::error::ErrorCode;
use crate::graphical_elements::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use j1939_async as j1939;
const FILE_CODE: u8 = 0x03;

// To convert C .h header to 1-bit data use these regexes:
// :.,.+15s/,//g
// :.,.+15s/ *\([01][01][01][01][01][01][01][01]\)\([01]*\)/    0b\1, 0b\2,/

#[rustfmt::skip]
pub const BLANK: &[u8;SMALL_ICON_NUM_BYTES] = &[
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
];

#[rustfmt::skip]
pub const BATTERY: &[u8;SMALL_ICON_NUM_BYTES] = &[
    0b00000000, 0b00000000,
    0b00000011, 0b11000000,
    0b00001111, 0b11110000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001000, 0b00010000,
    0b00001111, 0b11110000,
    0b00000000, 0b00000000,
];

#[rustfmt::skip]
pub const SOLAR: &[u8;SMALL_ICON_NUM_BYTES] = &[
    0b00000000, 0b00000000,
    0b00110001, 0b10001100,
    0b01111011, 0b11011110,
    0b01111011, 0b11011110,
    0b00110001, 0b10001100,
    0b00000000, 0b00000000,
    0b00110001, 0b10001100,
    0b01111011, 0b11011110,
    0b01111011, 0b11011110,
    0b00110001, 0b10001100,
    0b00000000, 0b00000000,
    0b00110001, 0b10001100,
    0b01111011, 0b11011110,
    0b01111011, 0b11011110,
    0b00110001, 0b10001100,
    0b00000000, 0b00000000,
];

#[rustfmt::skip]
pub const LIGHT: &[u8;SMALL_ICON_NUM_BYTES] = &[
    0b00000000, 0b00000000,
    0b00000100, 0b00100000,
    0b00100010, 0b01000100,
    0b00010000, 0b00001000,
    0b00001001, 0b10010000,
    0b00000011, 0b11000000,
    0b00000101, 0b11100000,
    0b00000101, 0b11100000,
    0b00000111, 0b11100000,
    0b00000011, 0b11000000,
    0b00000011, 0b11000000,
    0b00000001, 0b10000000,
    0b00000001, 0b10000000,
    0b00000001, 0b10000000,
    0b00000000, 0b00000000,
    0b00000000, 0b00000000,
];

pub const UNIT_CAPACITY: Unit = Unit::new_prefixable(0, 1.0 / 1_000_000.0, "Ah");
pub const UNIT_CURRENT: Unit = Unit::new_prefixable(0, 1.0 / 1_000_000.0, "A");
pub const UNIT_VOLT: Unit = Unit::new_non_prefixable(0, 1.0 / 1_000_000.0, "V");

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum ButtonMode {
    Scroll,
    X,
    Y,
    Numbers,
    Contrast,
}

impl ButtonMode {
    pub fn next(&mut self) {
        match self {
            Self::Scroll => *self = Self::X,
            Self::X => *self = Self::Y,
            Self::Y => *self = Self::Numbers,
            Self::Numbers => *self = Self::Contrast,
            Self::Contrast => *self = Self::Scroll,
        }
    }
}

#[derive(Copy, Clone, Debug, defmt::Format, PartialEq, Hash, PartialOrd, Ord)]
pub struct PdIdentifier {
    pub element_num: j1939::process_data::ElementNumber,
    pub ddi: j1939::process_data::Ddi,
}

impl PdIdentifier {
    pub fn new(element_num: u16, ddi: u16) -> Self {
        PdIdentifier { element_num, ddi }
    }
}

impl core::cmp::Eq for PdIdentifier {}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct PdState {
    ui_index: u8,
}

impl PdState {
    pub fn new(ui_index: u8) -> Self {
        PdState { ui_index }
    }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct PdRecord {
    id: PdIdentifier,
    state: PdState,
}

impl PdRecord {
    pub fn new(id: PdIdentifier, state: PdState) -> Self {
        PdRecord { id, state }
    }
}

pub struct PdStateStore {
    vec: heapless::Vec<PdRecord, 16>,
}

impl PdStateStore {
    pub fn new() -> Self {
        PdStateStore {
            vec: heapless::Vec::new(),
        }
    }

    pub fn insert(&mut self, id: PdIdentifier, state: PdState) -> Result<(), crate::Error> {
        if self.vec.len() >= self.vec.capacity() {
            return Err(crate::mkerr(FILE_CODE, crate::ErrorCode::TooBig, line!()));
        }

        match self.vec.binary_search_by_key(&id, |pd| pd.id) {
            Ok(_) => {
                return Err(crate::mkerr(
                    FILE_CODE,
                    crate::ErrorCode::Duplicate,
                    line!(),
                ));
            }
            Err(index) => {
                j1939::error::convert_if_err(
                    self.vec.insert(index, PdRecord::new(id, state)),
                    FILE_CODE,
                    ErrorCode::TooBig as u8,
                    line!(),
                )?;
            }
        }
        Ok(())
    }

    pub fn get(&self, id: &PdIdentifier) -> Option<&PdState> {
        match self.vec.binary_search_by_key(id, |pd| pd.id) {
            Ok(index) => Some(&self.vec[index].state),
            Err(_) => None,
        }
    }

    pub fn get_mut(&mut self, id: &PdIdentifier) -> Option<&mut PdState> {
        match self.vec.binary_search_by_key(id, |pd| pd.id) {
            Ok(index) => Some(&mut self.vec[index].state),
            Err(_) => None,
        }
    }
}

const MAX_NUMBERS: usize = 20;
const NUM_ROWS: u32 = 4;

pub struct Ui {
    pub display: crate::application::Display,
    display_reset_pin: crate::application::ResetPin,
    pub xoff: i32,
    pub yoff: i32,
    pub contrast: i16,
    pub selected: i32,
    pub top_visible: i32,
    pub mode: ButtonMode,
    pub numbers: heapless::Vec<NumberWithIcon<'static, 'static>, MAX_NUMBERS>,
    pub pd_state_store: PdStateStore,
}

impl Ui {
    pub fn new(
        display: crate::application::Display,
        display_reset_pin: crate::application::ResetPin,
    ) -> Self {
        Ui {
            display,
            display_reset_pin,
            xoff: 0,
            yoff: 0,
            contrast: 0,
            selected: -1,
            top_visible: 0,
            mode: ButtonMode::Scroll,
            numbers: heapless::Vec::new(),
            pd_state_store: PdStateStore::new(),
        }
    }
    pub fn add_pd_number_with_icon_and_unit(
        &mut self,
        element_num: j1939::process_data::ElementNumber,
        ddi: j1939::process_data::Ddi,
        image: &'static [u8; SMALL_ICON_NUM_BYTES],
        unit: &'static Unit,
    ) -> Result<(), crate::error::Error> {
        let pd_identifier = PdIdentifier::new(element_num, ddi);
        let ui_index = self.numbers.len() as u8;
        self.pd_state_store
            .insert(pd_identifier, PdState::new(ui_index))?;
        j1939::error::convert_if_err(
            self.numbers.push(NumberWithIcon::new(image, unit)),
            FILE_CODE,
            crate::error::ErrorCode::TooBig as u8,
            line!(),
        )?;

        Ok(())
    }

    pub fn reset_display<DELAY>(&mut self, delay: &mut DELAY) -> Result<(), Error>
    where
        DELAY: embedded_hal::delay::DelayNs,
    {
        self.display
            .reset(&mut self.display_reset_pin, delay)
            .map_err(|err| {
                j1939::error::mkerr_generic(FILE_CODE, ErrorCode::DisplayDraw as u8, line!(), err)
            })?;

        Ok(())
    }

    pub fn get_number_pos(&self, index: usize) -> Option<Rectangle> {
        if index > self.numbers.len() {
            return None;
        }
        let index = index as i32;
        if index < self.top_visible {
            return None;
        }
        if index >= self.top_visible + 4 {
            return None;
        }

        let (width, height) = self.display.get_dimensions();
        let item_height = height as u32 / NUM_ROWS;
        Some(Rectangle::new(
            Point::new(
                self.xoff,
                self.yoff + item_height as i32 * (index - self.top_visible),
            ),
            Size::new(width as u32, item_height),
        ))
    }

    pub async fn redraw(&mut self) -> Result<bool, Error> {
        self.selected = core::cmp::min(self.selected, self.numbers.len() as i32);
        self.selected = core::cmp::max(self.selected, -1);
        if self.selected > self.top_visible + 3 {
            self.top_visible = self.selected - 3;
        }
        if self.selected < self.top_visible && self.selected >= 0 {
            self.top_visible = self.selected;
        }

        self.display.clear();

        let mut opts = DrawOptions::new(true, true);

        /*defmt::println!(
            "Sel {} top {} dimensions {}",
            self.selected,
            self.top_visible,
            self.display.get_dimensions(),
        );*/
        for i in self.top_visible..(self.top_visible + 4) {
            if i >= self.numbers.len() as i32 {
                return Err(mkerr(FILE_CODE, ErrorCode::TooBig, line!()));
            }
            let rect = match self.get_number_pos(i as usize) {
                Some(rect) => rect,
                None => return Err(mkerr(FILE_CODE, ErrorCode::InvalidIndex, line!())),
            };

            opts.inverted = self.selected == i;
            let num = &mut self.numbers[i as usize];

            num.draw(&mut self.display, rect.top_left, rect.size, &opts)?;
        }

        self.display.flush().await.map_err(|err| {
            j1939::error::mkerr_generic(FILE_CODE, ErrorCode::DisplayDraw as u8, line!(), err)
        })?;

        Ok(true)
    }

    pub fn set_process_data(
        &mut self,
        pd: &j1939::process_data::ProcessData,
    ) -> Result<bool, Error> {
        let id = PdIdentifier::new(pd.element_number, pd.ddi);
        let ui_index = match self.pd_state_store.get(&id) {
            Some(&PdState { ui_index }) => ui_index as usize,
            None => {
                return Ok(false);
            }
        };
        if ui_index >= self.numbers.len() {
            return Ok(false);
        }
        let pos = self.get_number_pos(ui_index);

        let number = &mut self.numbers[ui_index];

        if pd.value == number.get_value() {
            return Ok(false);
        }
        number.set_value(pd.value);

        let pos = match pos {
            Some(pos) => pos,
            None => return Ok(false),
        };
        let opts = DrawOptions::new(false, self.selected == ui_index as i32);
        number.draw(&mut self.display, pos.top_left, pos.size, &opts)?;

        Ok(true)
    }
}
