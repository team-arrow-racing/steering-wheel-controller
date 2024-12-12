#![no_main]
#![no_std]
#![allow(clippy::transmute_ptr_to_ptr)]

mod canbus;
mod init;

// import tasks
use canbus::*;
use init::*;

// global logger
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal as hal;

use fdcan::{frame::RxFrameInfo, ExternalLoopbackMode, FdCan, NormalOperationMode};
use hal::{
    can::Can,
    gpio::ErasedPin,
    gpio::Output,
    i2c::I2c,
    independent_watchdog::IndependentWatchdog,
    pac::FDCAN1,
    prelude::*,
    stm32::{self, I2C1},
};
use ssd1306::{prelude::*, Ssd1306, mode::BufferedGraphicsMode};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text}
};

use rtic_monotonics::{
    systick::{ExtU64, Systick},
    Monotonic,
};

use heapless::String;
use core::fmt::write;

//CAN dependencies
use solar_car::com::{lighting::{lighting_header, LampsState}, horn::horn_header};
use solar_car::device::Device;

#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    use stm32h7xx_hal::gpio::{ExtiPin, Input, Pin};

    use super::*;

    type FdCanMode = ExternalLoopbackMode; // NormalOperationMode;
    type OLEDDisplay = Ssd1306<I2CInterface<I2c<I2C1>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

    #[shared]
    pub struct Shared {
        pub can: FdCan<Can<FDCAN1>, FdCanMode>,
    }

    #[local]
    pub struct Local {
        pub watchdog: IndependentWatchdog,
        pub led_ok: ErasedPin<Output>,
        pub led_warn: ErasedPin<Output>,
        pub led_error: ErasedPin<Output>,
        pub display: OLEDDisplay,
        pub buttons: InputButtons,
    }

    #[task(local = [watchdog])]
    async fn watchdog(cx: watchdog::Context) {
        loop {
            cx.local.watchdog.feed();
            Systick::delay(80_u64.millis()).await;
        }
    }

    extern "Rust" {
        #[init]
        fn init(mut cx: init::Context) -> (Shared, Local);

        #[task(binds = FDCAN1_IT0, priority = 2, shared = [can])]
        fn can_rx0_pending(mut cx: can_rx0_pending::Context);

        #[task(binds = FDCAN1_IT1, priority = 2, shared = [can])]
        fn can_rx1_pending(mut cx: can_rx1_pending::Context);

        #[task(priority = 1)]
        async fn can_receive(mut cx: can_receive::Context, frame: RxFrameInfo, buffer: [u8; 8]);
    }

    #[task(local = [display])]
    async fn update_display(cx: update_display::Context) {
        let mut num: u8 = 0;
        let mut output: String<32> = String::new();
        let display = cx.local.display;

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline("Hello World!", Point::zero(), text_style, Baseline::Top)
            .draw(display)
            .unwrap();
        display.flush().unwrap();

        Systick::delay(1000_u64.millis()).await;

        loop {
            output.clear();
            write(&mut output, format_args!("Hello {}!", num)).unwrap();

            display.clear_buffer();
            Text::with_baseline(output.as_str(), Point::zero(), text_style, Baseline::Top)
                .draw(display)
                .unwrap();
            display.flush().unwrap();

            num = num + 1;

            // TODO figure out why it crashes after 5 loops

            Systick::delay(1000_u64.millis()).await;
        }
    }

//triggers on driver input
#[task(binds = EXTI15_10, priority = 2, local = [buttons])]
fn button_pressed(cx: button_pressed::Context) {
    defmt::info!("Button Pressed!");
    let btn_indicator_left = &mut cx.local.buttons.btn_indicator_left;
    let btn_indicator_right = &mut cx.local.buttons.btn_indicator_right;
    let btn_pre_chr = &mut cx.local.buttons.btn_pre_chr;
    let btn_cruise = &mut cx.local.buttons.btn_cruise;
    let btn_horn = &mut cx.local.buttons.btn_horn;

    if btn_indicator_left.check_interrupt() {
        defmt::info!("Left Indicator Received!");
        btn_indicator_left.clear_interrupt_pending_bit();
        //SPAWN HANDLER
    }

    if btn_indicator_right.check_interrupt() {
        defmt::info!("Right Indicator Received!");
        btn_indicator_right.clear_interrupt_pending_bit();
        //SPAWN HANDLER
    }
    if btn_pre_chr.check_interrupt() {
        defmt::info!("Precharge Received!");
        btn_pre_chr.clear_interrupt_pending_bit();
        //SPAWN HANDLER
    }
    if btn_cruise.check_interrupt() {
        defmt::info!("Cruise Received!");
        btn_cruise.clear_interrupt_pending_bit();
        //SPAWN HANDLER
    }
    if btn_horn.check_interrupt() {
        defmt::info!("Horn Received!");
        btn_horn.clear_interrupt_pending_bit();
        //SPAWN HANDLER
    }
    
}


}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

defmt::timestamp!("{=u64:us}", {
    Systick::now().duration_since_epoch().to_micros()
});
