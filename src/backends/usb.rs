//! USB interface to a Tic motor driver board.

use core::time::Duration;
use std::prelude::rust_2024::*;

use nusb::transfer::{Control, ControlType, Recipient};

use crate::{
    base::{communication::TicCommunication, TicBase},
    TicCommand, TicHandlerError, TicProduct,
};

const VENDOR_ID: u16 = 0x1FFB;
const PRODUCT_ID_T825: u16 = 0x00B3;
const PRODUCT_ID_T834: u16 = 0x00B5;
const PRODUCT_ID_T500: u16 = 0x00BD;
const PRODUCT_ID_N825: u16 = 0x00C3;
const PRODUCT_ID_T249: u16 = 0x00C9;
const PRODUCT_ID_36V4: u16 = 0x00CB;

const PRODUCT_IDS: &[u16] = &[
    PRODUCT_ID_T825,
    PRODUCT_ID_T834,
    PRODUCT_ID_T500,
    PRODUCT_ID_N825,
    PRODUCT_ID_T249,
    PRODUCT_ID_36V4,
];

const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);


/// USB interface to a Tic board.
///
/// # WARNING:
/// USB support is **untested** and is considered **unstable**. This interface
/// can change at any time.
pub struct TicUsb {
    usb_interface: nusb::Interface,

    product: TicProduct,
}

impl TicUsb {
    /// List all Tic devices connected via USB to the computer.
    ///
    /// This function will automatically detect and query any devices attached
    /// to the system, if any other device is accessing them the device will
    /// not be listed.
    pub fn list_devices() -> Result<Vec<Self>, nusb::Error> {
        let device_list: Vec<Self> = nusb::list_devices()?
            .filter(|d| d.vendor_id() == VENDOR_ID)
            .filter(|d| PRODUCT_IDS.contains(&d.product_id()))
            .filter_map(|d| Self::new(d).ok())
            .collect();

        Ok(device_list)
    }

    /// Create a new USB interface to a Tic device.
    ///
    /// This function will return [`TicHandlerError::UsbInvalidDevice`] if the
    /// device passed to it is invalid. It is recommended to use
    /// [`Self::list_devices()`] to get a list of valid devices which can be
    /// used directly.
    pub fn new(device_info: nusb::DeviceInfo) -> Result<Self, TicHandlerError> {
        let device = device_info.open()?;
        let usb_interface = device.detach_and_claim_interface(0)?;

        let product = match device_info.product_id() {
            PRODUCT_ID_T825 => TicProduct::T825,
            PRODUCT_ID_T834 => TicProduct::T834,
            PRODUCT_ID_T500 => TicProduct::T500,
            PRODUCT_ID_N825 => TicProduct::N825,
            PRODUCT_ID_T249 => TicProduct::T249,
            PRODUCT_ID_36V4 => TicProduct::Tic36v4,
            id => return Err(TicHandlerError::UsbInvalidDevice(id)),
        };

        Ok(Self {
            usb_interface,
            product,
        })
    }

    /// This command writes a byte of data to the Tic’s settings (stored in
    /// non-volatile memory) at the specified offset. It is not available on
    /// the [`crate::TicSerial`] and [`crate::TicI2C`] interfaces.
    pub fn set_setting(&self, cmd: u8, data: u16, offset: u16) -> Result<(), TicHandlerError> {
        self.usb_interface.control_out_blocking(
            Control {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd,
                value: data,
                index: offset,
            },
            &[],
            DEFAULT_TIMEOUT
        )?;

        Ok(())
    }
}

impl TicCommunication for TicUsb {
    fn command_quick(&mut self, cmd: TicCommand) -> Result<(), TicHandlerError> {
        self.usb_interface.control_out_blocking(
            Control {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: 0,
                index: 0,
            },
            &[],
            DEFAULT_TIMEOUT
        )?;

        Ok(())
    }

    fn command_w7(&mut self, cmd: TicCommand, val: u8) -> Result<(), TicHandlerError> {
        self.usb_interface.control_out_blocking(
            Control {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: val as u16,
                index: 0,
            },
            &[],
            DEFAULT_TIMEOUT
        )?;

        Ok(())
    }

    fn command_w32(&mut self, cmd: TicCommand, val: u32) -> Result<(), TicHandlerError> {
        self.usb_interface.control_out_blocking(
            Control {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: (val & 0x00FF) as u16,
                index: ((val & 0xFF00) >> 16) as u16
            },
            &[],
            DEFAULT_TIMEOUT
        )?;

        Ok(())
    }

    fn block_read(
        &mut self,
        cmd: TicCommand,
        offset: u8,
        buffer: &mut [u8],
    ) -> Result<(), TicHandlerError> {
        self.usb_interface.control_in_blocking(
            Control {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: 0,
                index: offset as u16,
            },
            buffer,
            DEFAULT_TIMEOUT
        )?;

        Ok(())
    }
}

impl TicBase for TicUsb {
    fn product(&self) -> TicProduct {
        self.product
    }
}

impl From<nusb::Error> for TicHandlerError {
    fn from(err: nusb::Error) -> Self {
        Self::NusbError(err)
    }
}

impl From<nusb::transfer::TransferError> for TicHandlerError {
    fn from(err: nusb::transfer::TransferError) -> Self {
        Self::UsbTransferError(err)
    }
}
