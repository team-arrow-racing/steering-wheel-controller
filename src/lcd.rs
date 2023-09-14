use cortex_m::asm::nop;

use stm32l4xx_hal::{
    delay::{Delay, DelayCM},
    gpio::{Output, PushPull, PA0, PA1, PA4, PA5, PA6, PA7, PC0, PC1, PC2, PC3,PB7, PB6, PB1, PA8},
    prelude::*,
};

pub struct LCD {
    rs: PC2<Output<PushPull>>,
    en: PC3<Output<PushPull>>,
    d4: PA6<Output<PushPull>>,
    d5: PA7<Output<PushPull>>,
    d6: PA4<Output<PushPull>>,
    d7: PA5<Output<PushPull>>,
    delay: DelayCM,
}

impl LCD {
    pub fn new(
        rs: PC2<Output<PushPull>>,
        en: PC3<Output<PushPull>>,
        d4: PA6<Output<PushPull>>,
        d5: PA7<Output<PushPull>>,
        d6: PA4<Output<PushPull>>,
        d7: PA5<Output<PushPull>>,
        delay: DelayCM,
    ) -> LCD {
        LCD {
            rs,
            en,
            d4,
            d5,
            d6,
            d7,
            delay,
        }
    }

    /* -------------------- Private Functions -------------------- */

    /// Pulse enable
    fn _pulse_enable(&mut self) {
        self.en.set_high();
        nop();
        self.en.set_low();
    }

    /// Send half a byte to the LCD
    ///
    /// # Arguments
    ///
    /// * `nibble` - Nibble to send
    fn _send_nibble(&mut self, nibble: u8) {
        if ((nibble >> 3) & 0x01) == 0x01 {
            self.d7.set_high();
        } else {
            self.d7.set_low();
        }

        if ((nibble >> 2) & 0x01) == 0x01 {
            self.d6.set_high();
        } else {
            self.d6.set_low();
        }

        if ((nibble >> 1) & 0x01) == 0x01 {
            self.d5.set_high();
        } else {
            self.d5.set_low();
        }

        if ((nibble >> 0) & 0x01) == 0x01 {
            self.d4.set_high();
        } else {
            self.d4.set_low();
        }

        self._pulse_enable();
    }

    /* -------------------- Public Functions -------------------- */

    /// Initialize the LCD
    pub fn init(&mut self) {
        // Power on delay
        self.delay.delay_ms(100_u16);

        // Send command
        self.rs.set_low();

        // First nibble 0b0011
        self._send_nibble(0x03);
        self.delay.delay_us(4100_u16);

        // Second nibble 0b0011
        self._pulse_enable();
        self.delay.delay_us(100_u16);

        // Third nibble 0b0011
        self._pulse_enable();
        self.delay.delay_us(100_u16);

        // Configure LCD in 4-bit mode
        self._send_nibble(0x02);
        self.delay.delay_us(100_u16);

        // Function set to configure the interface, number of lines and the font
        self.send_cmd(0x28);
        self.delay.delay_us(53_u16);

        // Display off
        self.send_cmd(0x08);
        self.delay.delay_us(53_u16);

        // Clear display (demands a longer delay)
        self.send_cmd(0x01);
        self.delay.delay_us(3000_u16);

        // Entry mode set
        self.send_cmd(0x06);
        self.delay.delay_us(53_u16);

        // Display on
        self.send_cmd(0x0C);
        self.delay.delay_us(53_u16);
    }

    /// Send command to the LCD
    ///
    /// # Arguments
    ///
    /// * `cmd` - Command to send
    pub fn send_cmd(&mut self, cmd: u8) {
        self.rs.set_low();

        let higher_nibble = (cmd >> 4) & 0x0F;
        let lower_nibble = (cmd >> 0) & 0x0F;

        self._send_nibble(higher_nibble);
        self._send_nibble(lower_nibble);
    }

    pub fn clear_display(&mut self) {
        self.send_cmd(0x01);
        self.delay.delay_us(3000_u16);
    }

    pub fn set_position(
        &mut self,
        x: u8,
        y: u8
    ) {
        match (x,y) {
            (0..=15, 0) => {
                self.send_cmd(0x80 | x);
                self.delay.delay_us(53_u16)
            },
            (0..=15, 1) => {
                self.send_cmd(0x80 | (x + 0x40));
                self.delay.delay_us(53_u16)
            },
            _ => {}
        }
    }

    /// Send data to the LCD
    ///
    /// # Arguments
    ///
    /// * `data` - Byte to send
    #[allow(dead_code)]
    pub fn send_data(&mut self, data: u8) {
        self.rs.set_high();

        let higher_nibble = (data >> 4) & 0x0F;
        let lower_nibble = (data >> 0) & 0x0F;

        self._send_nibble(higher_nibble);
        self._send_nibble(lower_nibble);

        self.delay.delay_us(40_u16);
    }

    /// Send a string to the LCD
    ///
    /// # Arguments
    ///
    /// * `string` - String to send
    #[allow(dead_code)]
    pub fn send_string(&mut self, string: &str) {
        for byte in string.chars() {
            self.send_data(byte as u8);
        }
    }
}
