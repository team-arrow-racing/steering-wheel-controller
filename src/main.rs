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
#[rtic::app(device = stm32h7xx_hal::pac, dispatchers = [UART4, SPI1])]
mod app {
    use display_interface_parallel_gpio::{Generic16BitBus, PGpio16BitInterface};
    use embedded_graphics::pixelcolor::Rgb666;
    use embedded_hal::digital::v2::OutputPin;
    use mipidsi::{models::{ILI9341Rgb666, ILI9486Rgb666}, Display};
    use stm32h7xx_hal::{gpio::{Pin, PushPull}, pac::SPI3};

    use super::*;

    type FdCanMode = ExternalLoopbackMode; // NormalOperationMode;

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
        // pain
        pub display: Display<PGpio16BitInterface<Generic16BitBus<Pin<'D', 15, Output>, Pin<'D', 14, Output>, Pin<'D', 13, Output>, Pin<'D', 12, Output>, Pin<'D', 11, Output>, Pin<'E', 2, Output>, Pin<'B', 2, Output>, Pin<'B', 6, Output>, Pin<'A', 15, Output>, Pin<'B', 8, Output>, Pin<'B', 9, Output>, Pin<'E', 7, Output>, Pin<'E', 10, Output>, Pin<'E', 12, Output>, Pin<'E', 14, Output>, Pin<'E', 15, Output>>, Pin<'B', 4, Output>, Pin<'B', 15, Output>>, ILI9486Rgb666, Pin<'C', 6, Output>>
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
        .text_color(Rgb666::RED)
        .build();

        Text::with_baseline("Hello World!", Point::new(10, 100), text_style, Baseline::Top)
            .draw(display)
            .unwrap();

        Systick::delay(1000_u64.millis()).await;

        loop {
            defmt::info!("printing to display");
            write(&mut output, format_args!("Hello {}!", num)).unwrap();

            display.set_pixels(10, 100, 250, 30, core::iter::repeat(Rgb666::WHITE).take(20*150)).unwrap();

            Text::with_baseline(output.as_str(), Point::new(10, 100), text_style, Baseline::Top)
                .draw(display)
                .unwrap();

            num = num + 1;
            output.clear();
            
            Systick::delay(1000_u64.millis()).await;
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
