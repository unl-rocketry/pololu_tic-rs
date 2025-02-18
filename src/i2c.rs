//! I2C interface to a Tic motor driver board.

use embedded_hal::i2c::I2c;

use crate::{base::{communication::TicCommunication, TicBase}, TicCommand, TicProduct};

/// I2C interface to a Tic board.
pub struct TicI2C<I2C> {
    address: u8,
    i2c: I2C,
    product: TicProduct,
    last_error: u8,
}

impl<I2C: I2c> TicI2C<I2C> {
    const DEFAULT_ADDR: u8 = 14;

    pub fn new_default(i2c: I2C, product: TicProduct) -> Self {
        Self {
            address: Self::DEFAULT_ADDR,
            i2c,
            product,
            last_error: 0,
        }
    }

    pub fn new_with_address(i2c: I2C, product: TicProduct, address: u8) -> Self {
        Self {
            address,
            i2c,
            product,
            last_error: 0,
        }
    }

    pub fn get_address(&self) -> u8 {
        self.address
    }
}

// TODO: Properly error handle this stuff!!!
impl<I2C: I2c> TicCommunication for TicI2C<I2C> {
    fn command_quick(&mut self, cmd: TicCommand) {
        self.i2c.write(self.address, &[cmd as u8]).unwrap();
    }

    fn command_w32(&mut self, cmd: TicCommand, val: u32) {
        let mut data = [0u8; 5];
        data[0] = cmd as u8;
        data[1..5].copy_from_slice(&val.to_le_bytes());
        self.i2c.write(self.address, &data).unwrap();
    }

    fn command_w7(&mut self, cmd: TicCommand, val: u8) {
        let mut data = [0u8; 2];
        data[0] = cmd as u8;
        data[1] = val & 0x7F;
        self.i2c.write(self.address, &data).unwrap();
    }

    fn get_segment(&mut self, cmd: TicCommand, offset: u8, buffer: &mut [u8]) {
        let data = [cmd as u8, offset];
        if self.i2c.write(self.address, &data).is_err() {
            // Something went wrong!
            return;
        }

        self.i2c.read(self.address, buffer).unwrap();
    }
}

impl<I2C: I2c> TicBase for TicI2C<I2C> {
    fn product(&self) -> TicProduct {
        self.product
    }

    fn get_last_error(&self) -> u8 {
        self.last_error
    }
}
