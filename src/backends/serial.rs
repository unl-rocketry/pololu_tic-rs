//! Serial interface to a Tic motor driver board.

use embedded_hal::delay::DelayNs;
use embedded_io::{Error, Read, Write};

use crate::{
    base::{communication::TicCommunication, TicBase},
    Command, HandlerError, Product,
};

/// UART serial interface to a Tic board.
pub struct Serial<S, D: DelayNs> {
    stream: S,
    product: Product,
    device_number: u8,
    delay: D,
}

impl<S: Write + Read, D: DelayNs> Serial<S, D> {
    pub fn new_default(stream: S, product: Product, delay: D) -> Self {
        Self {
            stream,
            product,
            device_number: 255,
            delay,
        }
    }

    pub fn new_with_number(stream: S, product: Product, delay: D, device_number: u8) -> Self {
        Self {
            stream,
            product,
            device_number,
            delay,
        }
    }

    fn send_command_header(&mut self, cmd: Command) -> Result<(), HandlerError> {
        match self.device_number {
            255 => self.stream.write_all(&[cmd as u8])
                .map_err(|e| HandlerError::StreamError(e.kind()))?,
            _ => {
                self.stream.write_all(&[0xAA]).map_err(|e| HandlerError::StreamError(e.kind()))?;
                self.serial_w7(self.device_number)?;
                self.serial_w7(cmd as u8)?;
            }
        }

        Ok(())
    }

    fn serial_w7(&mut self, val: u8) -> Result<(), HandlerError> {
        self.stream.write_all(&[val & 0x7F])
            .map_err(|e| HandlerError::StreamError(e.kind()))?;
        Ok(())
    }
}

impl<S: Write + Read, D: DelayNs> TicCommunication for Serial<S, D> {
    fn command_quick(&mut self, cmd: Command) -> Result<(), HandlerError> {
        self.send_command_header(cmd)?;

        Ok(())
    }

    fn command_w32(&mut self, cmd: Command, val: u32) -> Result<(), HandlerError> {
        self.send_command_header(cmd)?;

        // byte with MSbs:
        // bit 0 = MSb of first (least significant) data byte
        // bit 1 = MSb of second data byte
        // bit 2 = MSb of third data byte
        // bit 3 = MSb of fourth (most significant) data byte
        self.serial_w7(
            (((val >> 7) & 1) | ((val >> 14) & 2) | ((val >> 21) & 4) | ((val >> 28) & 8)) as u8,
        )?;

        self.serial_w7((val) as u8)?; // least significant byte with MSb cleared
        self.serial_w7((val >> 8) as u8)?;
        self.serial_w7((val >> 16) as u8)?;
        self.serial_w7((val >> 24) as u8)?; // most significant byte with MSb cleared

        Ok(())
    }

    fn command_w7(&mut self, cmd: Command, val: u8) -> Result<(), HandlerError> {
        self.send_command_header(cmd)?;
        self.serial_w7(val)?;

        Ok(())
    }

    fn block_read(
        &mut self,
        cmd: Command,
        offset: u8,
        buffer: &mut [u8],
    ) -> Result<(), HandlerError> {
        self.send_command_header(cmd)?;
        self.serial_w7(offset & 0x7F)?;
        self.serial_w7((buffer.len() | offset as usize >> 1 & 0x40) as u8)?;

        self.stream
            .read_exact(buffer)
            .map_err(|_| HandlerError::StreamError(embedded_io::ErrorKind::Other))?;

        Ok(())
    }
}

impl<S: Write + Read, D: DelayNs> TicBase for Serial<S, D> {
    fn product(&self) -> Product {
        self.product
    }

    fn delay(&mut self, delay: core::time::Duration) {
        self.delay.delay_ns(delay.as_nanos() as u32);
    }
}

impl From<embedded_io::ErrorKind> for HandlerError {
    fn from(err: embedded_io::ErrorKind) -> Self {
        Self::StreamError(err)
    }
}
