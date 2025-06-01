use embedded_graphics::{
    image::{Image, ImageRaw},
    mono_font::MonoTextStyle,
    //image::{Image, ImageRawLE},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::*,
    text::{renderer::CharacterStyle, Baseline, Text},
};
const FILE_CODE: u8 = 0x04;

pub const SMALL_ICON_SIZE: usize = 16;
pub const SMALL_ICON_NUM_BYTES: usize = SMALL_ICON_SIZE * SMALL_ICON_SIZE / 8;

use crate::error::*;
use embedded_graphics_core::geometry::{Point, Size};

pub struct DrawOptions {
    pub full_paint: bool, // True for a full paint required. False, only need to repaint updates.
    pub inverted: bool,
}

impl DrawOptions {
    pub fn new(full_paint: bool, inverted: bool) -> Self {
        DrawOptions {
            full_paint,
            inverted,
        }
    }
}

/// A Drawable that does not know where it want's to be drawn.
pub trait DrawableSomewhere {
    /// The pixel color type.
    type Color: embedded_graphics_core::pixelcolor::PixelColor;
    type Output;

    /// Draw the graphics object using the supplied DrawTarget.
    fn draw<D>(
        &self,
        target: &mut D,
        pos: Point,
        size: Size,
        _options: &DrawOptions,
    ) -> Result<Self::Output, Error>
    where
        D: embedded_graphics_core::draw_target::DrawTarget<Color = Self::Color>;
}

pub fn drwerr<D>(line: u32, detail: D) -> Error {
    j1939::error::mkerr_ptr(
        FILE_CODE,
        ErrorCode::DisplayDraw as u8,
        line,
        &detail as *const D as *const u8,
        core::mem::size_of::<D>(),
    )
}

pub trait Element {}

#[derive(Copy, Clone, Debug, defmt::Format)]

pub struct UnitSpec {
    offset: i32,
    scale: f32,
    string: &'static str,
}

const NO_PREFIX: &str = "";
const BIG_SI_PREFIXES: [&str; 3] = ["k", "M", "G"];
const SMALL_SI_PREFIXES: [&str; 4] = ["m", "u", "n", "p"];

impl UnitSpec {
    pub const fn new(offset: i32, scale: f32, string: &'static str) -> Self {
        UnitSpec {
            offset,
            scale,
            string,
        }
    }
    pub fn get_offset(&self) -> i32 {
        self.offset
    }
    pub fn get_scale(&self) -> f32 {
        self.scale
    }
    pub fn get_string(&self) -> &'static str {
        self.string
    }
    pub fn value(&self, value: i32) -> f32 {
        (value as f32 + self.offset as f32) * self.scale
    }

    pub fn to_string<const CAP: usize>(&self, value: i32) -> Result<heapless::String<CAP>, ()> {
        let value = self.value(value);
        let mut ret = heapless::String::<CAP>::new();
        let value = tfmt::Convert::<CAP>::from_f32(value, 2)?;
        ret.push_str(value.as_str())?;
        ret.push_str(self.string)?;
        Ok(ret)
    }

    pub fn to_string_prefixed<const CAP: usize>(
        &self,
        value: i32,
    ) -> Result<heapless::String<CAP>, ()> {
        let mut value = self.value(value);
        let l10 = libm::ceilf(libm::log10f(libm::fabsf(value)));
        let mut prefix = NO_PREFIX;
        if !l10.is_finite() {
            // Do nothing. Could be zero
        } else if l10 >= 4.0 {
            let mut prefix_idx = (l10 as i32 - 1) / 3 - 1;
            if prefix_idx >= BIG_SI_PREFIXES.len() as i32 {
                prefix_idx = BIG_SI_PREFIXES.len() as i32 - 1;
            }
            prefix = BIG_SI_PREFIXES[prefix_idx as usize];
            value *= libm::powf(10.0, -(3 * (prefix_idx + 1)) as f32);
        } else if l10 <= -1.0 {
            let mut prefix_idx = (-(l10 as i32) + 3) / 3 - 1;
            if prefix_idx >= SMALL_SI_PREFIXES.len() as i32 {
                prefix_idx = SMALL_SI_PREFIXES.len() as i32 - 1;
            }
            prefix = SMALL_SI_PREFIXES[prefix_idx as usize];
            value *= libm::powf(10.0, (3 * (prefix_idx + 1)) as f32);
        }

        let mut ret = heapless::String::<CAP>::new();
        let value = tfmt::Convert::<CAP>::from_f32(value, 2)?;
        ret.push_str(value.as_str())?;
        ret.push_str(prefix)?;
        ret.push_str(self.string)?;
        Ok(ret)
    }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Unit {
    NonPrefixable(UnitSpec),
    Prefixable(UnitSpec), // Allowed to add standard metric scale prefixes.
}

impl Unit {
    pub const fn new_non_prefixable(offset: i32, scale: f32, string: &'static str) -> Self {
        Unit::NonPrefixable(UnitSpec::new(offset, scale, string))
    }

    pub const fn new_prefixable(offset: i32, scale: f32, string: &'static str) -> Self {
        Unit::Prefixable(UnitSpec::new(offset, scale, string))
    }

    pub fn get_unit_spec(&self) -> &UnitSpec {
        match self {
            Unit::NonPrefixable(spec) => spec,
            Unit::Prefixable(spec) => spec,
        }
    }
    pub fn to_string<const CAP: usize>(&self, value: i32) -> Result<heapless::String<CAP>, ()> {
        match self {
            Unit::NonPrefixable(spec) => spec.to_string(value),
            Unit::Prefixable(spec) => spec.to_string_prefixed(value),
        }
    }

    pub fn get_offset(&self) -> i32 {
        self.get_unit_spec().offset
    }
    pub fn get_scale(&self) -> f32 {
        self.get_unit_spec().scale
    }
    pub fn get_string(&self) -> &'static str {
        self.get_unit_spec().string
    }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct NumberWithIcon<'a, 'b> {
    image: &'a [u8; 32],
    unit: &'b Unit,
    value: i32,
}

impl<'a, 'b> NumberWithIcon<'a, 'b> {
    pub fn new(image: &'a [u8; SMALL_ICON_NUM_BYTES], unit: &'b Unit) -> Self {
        NumberWithIcon {
            image,
            unit,
            value: 0,
        }
    }
    pub fn get_value(&self) -> i32 {
        self.value
    }
    pub fn set_value(&mut self, value: i32) {
        self.value = value;
    }
}

impl<'a, 'b> DrawableSomewhere for NumberWithIcon<'a, 'b> {
    type Color = embedded_graphics_core::pixelcolor::BinaryColor;
    type Output = ();

    fn draw<D>(
        &self,
        target: &mut D,
        pos: Point,
        _size: Size,
        options: &DrawOptions,
    ) -> Result<Self::Output, Error>
    where
        D: embedded_graphics_core::draw_target::DrawTarget<Color = Self::Color>,
    {
        let invert_all = false;

        let mut big = MonoTextStyle::new(
            #[cfg(not(feature = "sh1108_128_160"))]
            &embedded_graphics::mono_font::ascii::FONT_7X14,
            #[cfg(feature = "sh1108_128_160")]
            &embedded_graphics::mono_font::ascii::FONT_10X20,
            if invert_all {
                BinaryColor::Off
            } else {
                BinaryColor::On
            },
        );

        //let _e = crate::error::mkerrd(FILE_CODE, crate::error::ErrorCode::TooBig, line!(), );
        //let invert_all = options.inverted;
        if options.full_paint {
            if invert_all {
                let style = PrimitiveStyleBuilder::new()
                    .stroke_color(BinaryColor::On)
                    .stroke_width(1)
                    .fill_color(BinaryColor::On)
                    .build();
                Rectangle::new(pos, Size::new(128, 16))
                    .into_styled(style)
                    .draw(target)
                    .map_err(|e| drwerr(line!(), e))?;
            }

            if options.inverted {
                let mut image = self.image.clone();
                for i in 0..image.len() {
                    image[i] = !image[i];
                }

                match ImageRaw::<BinaryColor>::new(&image, Size::new(16, 16)) {
                    Ok(raw_image) => {
                        Image::new(&raw_image, pos)
                            .draw(target)
                            .map_err(|e| drwerr(line!(), e))?;
                    }
                    Err(_) => {}
                }
            } else {
                match ImageRaw::<BinaryColor>::new(self.image, Size::new(16, 16)) {
                    Ok(raw_image) => {
                        Image::new(&raw_image, pos)
                            .draw(target)
                            .map_err(|e| drwerr(line!(), e))?;
                    }
                    Err(_) => {}
                }
            }
        } else {
            big.set_background_color(Some(BinaryColor::Off));
        }

        match self.unit.to_string::<12>(self.value) {
            Ok(s) => {
                Text::with_baseline(s.as_str(), pos + Point::new(18, 0), big, Baseline::Top)
                    .draw(target)
                    .map_err(|e| drwerr(line!(), e))?;
            }
            Err(_) => {
                Text::with_baseline("Error", pos + Point::new(18, 0), big, Baseline::Top)
                    .draw(target)
                    .map_err(|e| drwerr(line!(), e))?;
            }
        }
        return Ok(());
    }
}
