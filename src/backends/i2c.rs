//! I2C interface to a Tic motor driver board.

use embedded_hal::{delay::DelayNs, i2c::{Error, I2c}};

use crate::{
    base::{communication::TicCommunication, TicBase},
    TicCommand, TicHandlerError, TicProduct,
};

/// I2C interface to a Tic board.
pub struct TicI2C<I2C: I2c, D: DelayNs> {
    address: u8,
    i2c: I2C,
    product: TicProduct,
    delay: D,
}

impl<I2C: I2c, D: DelayNs> TicI2C<I2C, D> {
    const DEFAULT_ADDR: u8 = 14;

    pub const fn new_default(i2c: I2C, product: TicProduct, delay: D) -> Self {
        Self {
            address: Self::DEFAULT_ADDR,
            i2c,
            product,
            delay,
        }
    }

    pub const fn new_with_address(i2c: I2C, product: TicProduct, delay: D, address: u8) -> Self {
        Self {
            address,
            i2c,
            product,
            delay,
        }
    }

    pub const fn get_address(&self) -> u8 {
        self.address
    }
}

impl<I2C: I2c, D: DelayNs> TicCommunication for TicI2C<I2C, D> {
    fn command_quick(&mut self, cmd: TicCommand) -> Result<(), TicHandlerError> {
        self.i2c
            .write(self.address, &[cmd as u8])
            .map_err(|e| TicHandlerError::I2cError(e.kind()))?;
        Ok(())
    }

    fn command_w32(&mut self, cmd: TicCommand, val: u32) -> Result<(), TicHandlerError> {
        let mut data = [0u8; 5];
        data[0] = cmd as u8;
        data[1..5].copy_from_slice(&val.to_le_bytes());
        self.i2c
            .write(self.address, &data)
            .map_err(|e| TicHandlerError::I2cError(e.kind()))?;
        Ok(())
    }

    fn command_w7(&mut self, cmd: TicCommand, val: u8) -> Result<(), TicHandlerError> {
        let mut data = [0u8; 2];
        data[0] = cmd as u8;
        data[1] = val & 0x7F;
        self.i2c
            .write(self.address, &data)
            .map_err(|e| TicHandlerError::I2cError(e.kind()))?;
        Ok(())
    }

    fn block_read(
        &mut self,
        cmd: TicCommand,
        offset: u8,
        buffer: &mut [u8],
    ) -> Result<(), TicHandlerError> {
        let data = [cmd as u8, offset];
        self.i2c
            .write_read(self.address, &data, buffer)
            .map_err(|e| TicHandlerError::I2cError(e.kind()))?;
        Ok(())
    }
}

impl<I2C: I2c, D: DelayNs> TicBase for TicI2C<I2C, D> {
    fn product(&self) -> TicProduct {
        self.product
    }

    fn delay(&mut self, delay: core::time::Duration) {
        self.delay.delay_ns(delay.as_nanos() as u32);
    }
}

impl From<embedded_hal::i2c::ErrorKind> for TicHandlerError {
    fn from(err: embedded_hal::i2c::ErrorKind) -> Self {
        Self::I2cError(err)
    }
}
