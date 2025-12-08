//! Functionality relating to the USB driver.
//!
#![doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>The USB driver is <b>untested, unstable</b>, and may change at <b>any time</b>!</p></div>\n\n<br/>\n\n"]

use core::time::Duration;
use std::prelude::rust_2024::*;

use nusb::{transfer::{ControlIn, ControlOut, ControlType, Recipient}, DeviceId, MaybeFuture};

use crate::{
    base::{communication::TicCommunication, TicBase},
    Command, HandlerError, Product,
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


/// List all Tic devices connected via USB to the computer.
///
/// This function will automatically detect and query any Tic devices attached
/// to the system, and return a list of [`UsbInfo`] structs which can be queried
/// further to find the device you want.
///
/// # Example:
/// ```rust,ignore
/// let devices = list_devices().expect("Getting devices failed!");
///
/// // Print some information about the detected devices
/// for device in devices {
///     println!(
///         "{}: {}, {}",
///         device.product(),
///         device.serial_number(),
///         device.firmware_revision(),
///     );
/// }
/// ```
pub async fn list_devices() -> Result<Vec<UsbInfo>, HandlerError> {
    let device_list = nusb::list_devices().await?
        .filter(|d| d.vendor_id() == VENDOR_ID)
        .filter(|d| PRODUCT_IDS.contains(&d.product_id()))
        .filter_map(|d| UsbInfo::new(d).ok())
        .collect();

    Ok(device_list)
}


/// Information about a Tic board attached via USB.
///
/// Constructing this struct does not actually initialize the device. In order
/// to use the device after constructing it, call [`Self::init()`].
pub struct UsbInfo {
    usb_device_info: nusb::DeviceInfo,

    serial_number: String,
    firmware_revision: u16,
    unique_id: DeviceId,

    product: Product,
}


impl UsbInfo {
    /// Create a new USB interface to a Tic device. To actually use the device,
    /// you must call [`Self::init()`] after creating it. Otherwise, only some
    /// basic information is available.
    ///
    /// This function will return [`HandlerError::UsbInvalidDevice`] if the
    /// device passed to it is not a Tic device. It is recommended to use
    /// [`list_devices()`] to get a list of valid devices which can be used
    /// directly.
    pub fn new(device_info: nusb::DeviceInfo) -> Result<Self, HandlerError> {
        let product = match device_info.product_id() {
            PRODUCT_ID_T825 => Product::T825,
            PRODUCT_ID_T834 => Product::T834,
            PRODUCT_ID_T500 => Product::T500,
            PRODUCT_ID_N825 => Product::N825,
            PRODUCT_ID_T249 => Product::T249,
            PRODUCT_ID_36V4 => Product::Tic36v4,
            id => return Err(HandlerError::UsbInvalidDevice(id)),
        };

        let serial_number = device_info.serial_number().unwrap_or("").to_owned();
        let firmware_revision = device_info.device_version();
        let os_id = device_info.id();

        // Create the device without an interface for now.
        Ok(Self {
            usb_device_info: device_info,
            product,
            serial_number,
            firmware_revision,
            unique_id: os_id,
        })
    }

    /// Initialize the device, returning a [`Usb`] device which is able to be
    /// operated on.
    pub async fn init(&mut self) -> Result<Usb, HandlerError> {
        let device = self.usb_device_info.open().await?;
        let usb_interface = device.detach_and_claim_interface(0).await?;

        Ok(Usb {
            usb_interface,
            product: self.product,
        })
    }

    /// Returns the [`Product`] that the device is.
    pub fn product(&self) -> Product {
        self.product
    }

    /// Returns the serial number of the device.
    ///
    /// This is useful for identifying a specific device among many devices on a
    /// system.
    pub fn serial_number(&self) -> &String {
        &self.serial_number
    }

    /// Returns the firmware revision of the device.
    pub fn firmware_revision(&self) -> u16 {
        self.firmware_revision
    }

    pub fn device_id(&self) -> DeviceId {
        self.unique_id
    }
}


/// USB interface to a Tic board.
pub struct Usb {
    usb_interface: nusb::Interface,

    product: Product,
}

impl Usb {
    /// This command writes a byte of data to the Tic’s settings (stored in
    /// non-volatile memory) at the specified offset.
    ///
    /// It is not available on the [`crate::Serial`] and [`crate::I2c`]
    /// interfaces.
    pub async fn set_setting(&mut self, cmd: u8, data: u16, offset: u16) -> Result<(), HandlerError> {
        self.usb_interface.control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd,
                value: data,
                index: offset,
                data: &[],
            },
            DEFAULT_TIMEOUT
        ).await?;

        Ok(())
    }
}

impl TicCommunication for Usb {
    fn command_quick(&mut self, cmd: Command) -> Result<(), HandlerError> {
        self.usb_interface.control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: 0,
                index: 0,
                data: &[],
            },
            DEFAULT_TIMEOUT
        ).wait()?;

        Ok(())
    }

    fn command_w7(&mut self, cmd: Command, val: u8) -> Result<(), HandlerError> {
        self.usb_interface.control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: val as u16,
                index: 0,
                data: &[],
            },
            DEFAULT_TIMEOUT
        ).wait()?;

        Ok(())
    }

    fn command_w32(&mut self, cmd: Command, val: u32) -> Result<(), HandlerError> {
        self.usb_interface.control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: (val & 0x00FF) as u16,
                index: ((val & 0xFF00) >> 16) as u16,
                data: &[],
            },
            DEFAULT_TIMEOUT
        ).wait()?;

        Ok(())
    }

    fn block_read(
        &mut self,
        cmd: Command,
        offset: u8,
        buffer: &mut [u8],
    ) -> Result<(), HandlerError> {
        let result = self.usb_interface.control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: cmd as u8,
                value: 0,
                index: offset as u16,
                length: buffer.len() as u16,
            },
            DEFAULT_TIMEOUT
        ).wait()?;

        buffer.copy_from_slice(&result);

        Ok(())
    }
}

impl TicBase for Usb {
    fn product(&self) -> Product {
        self.product
    }

    fn delay(&mut self, delay: core::time::Duration) {
        std::thread::sleep(delay)
    }
}

impl From<nusb::Error> for HandlerError {
    fn from(err: nusb::Error) -> Self {
        Self::NusbError(err)
    }
}

impl From<nusb::transfer::TransferError> for HandlerError {
    fn from(err: nusb::transfer::TransferError) -> Self {
        Self::UsbTransferError(err)
    }
}
