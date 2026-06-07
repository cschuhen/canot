//! CAN interface initialization — configures filter, bitrate, starts the bus,
//! wraps it with buffered TX/RX, and initializes global statics.
//!
//! This module contains the core logic for setting up the CAN peripheral, extracted from
//! `main.rs` to keep the RTIC app definition lean.

use embassy_stm32::can;
use embassy_sync::{mutex::Mutex, once_lock::OnceLock};
use static_cell::StaticCell;

use crate::bsp::BufferedCanErrorSender;
use crate::{FILE_CODE, MAIN_ERROR_SENDER};

/// CAN transmit buffer size
pub const CAN_TX_BUF_SIZE: usize = 8;

/// CAN receive buffer size
pub const CAN_RX_BUF_SIZE: usize = 20;

/// Global static for the buffered CAN interface
static CAN_IFACE: OnceLock<can::BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>> =
    OnceLock::new();

/// Initialize the CAN interface.
///
/// Configures extended filter (accept all into FIFO1), sets 250k bps, starts
/// in NormalOperationMode, wraps with buffered TX/RX buffers, and initializes
/// the global `CAN_IFACE` and `MAIN_ERROR_SENDER` statics.
///
/// Returns `(can_receiver, can_sender, error_sender)` for downstream use (e.g., `MonitorApp::new`).
pub fn init_can(
    mut can_iface: can::CanConfigurator<'static>,
) -> (
    can::BufferedCanReceiver,
    can::BufferedCanSender,
    BufferedCanErrorSender,
) {
    use j1939_async::error::SendError;

    // Setup CAN filter — accept all extended frames into FIFO1
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
    let can_receiver = can_iface.reader();
    let can_sender = can_iface.writer();
    let mut error_sender = BufferedCanErrorSender::new(can_sender.clone());

    // Initialize CAN interface as global static
    CAN_IFACE.get_or_init(|| can_iface);

    error_sender.report(
        FILE_CODE,
        crate::error::ErrorCode::CheckPoint as u8,
        line!(),
    );
    // Initialize main error sender as global static
    MAIN_ERROR_SENDER.get_or_init(|| Mutex::new(error_sender.clone()));

    (can_receiver, can_sender, error_sender)
}
