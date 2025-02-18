use crate::{base::TicBase, TicProduct};

struct TicSerial {
    product: TicProduct,
    device_number: u8,
    last_error: u8;
}

impl TicSerial {
    fn get_device_number(&self) -> u8 {
        self.device_number
    }
}

impl TicBase for TicSerial {
    fn send_command_header(&mut self, cmd: crate::TicCommand) {
        if self.device_number == 255 {
            // Compact protocol
            _stream->write((uint8_t)cmd);
        } else {
            // Pololu protocol
            _stream->write((uint8_t)0xAA);
            serialW7(_deviceNumber);
            serialW7((uint8_t)cmd);
        }
        self.lastError = 0;
    }

    fn command_quick(&mut self, cmd: crate::TicCommand) {
        todo!()
    }

    fn command_w32(&mut self, cmd: crate::TicCommand, val: u32) {
        todo!()
    }

    fn command_w7(&mut self, cmd: crate::TicCommand, val: u8) {
        todo!()
    }

    fn get_segment(&mut self, cmd: crate::TicCommand, offset: u8, length: u8, buffer: &mut [u8]) {
        todo!()
    }

    fn set_product(&mut self, product: TicProduct) {
        todo!()
    }

    fn product(&self) -> TicProduct {
        self.product
    }

    fn get_last_error() -> u8 {
        todo!()
    }
}
