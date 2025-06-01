#[derive(Copy, Clone, defmt::Format)]
pub enum ErrorCode {
    CheckPoint = 0,
    NoDevice = 1,
    I2CRead = 2,
    NoSpace = 3,
    SpawnError = 4,
    StoreI2c = 5,
    BmeError = 6,
    PdMissing = 7,
    IndexMissing = 8,
    FlashWrite = 9,
    FlashRead = 10,
    FlashErase = 11,
    RunFail = 12,
    SendEvent = 13,
    TooBig = 14,
    Duplicate = 15,
    DisplayDraw = 16,
    InvalidIndex = 17,

    CAN = 20,
}

pub use j1939_async::error::Error;
pub use j1939_async::error::SendError;

pub fn mkerr(file_code: u8, code: crate::error::ErrorCode, line: u32) -> Error {
    j1939_async::error::mkerr_base(file_code, code as u8, line, 0)
}

pub fn mkerrd(file_code: u8, code: crate::error::ErrorCode, line: u32, detail: u32) -> Error {
    j1939_async::error::mkerr_base(file_code, code as u8, line, detail)
}

/*impl From<eeprom24x::Error<embassy_stm32::i2c::Error>> for Error {
    fn from(_err: eeprom24x::Error<embassy_stm32::i2c::Error>) -> Self {
        mkerr(ErrorCode::StoreI2c, 0)
    }
}

impl From<j1939::error::Error> for Error {
    fn from(err: j1939::error::Error) -> Self {
        Error(err)
    }
}
*/
