use crate::app::{init, update_display, watchdog, Local, Shared};
use crate::hal::{
    gpio::Speed,
    independent_watchdog::IndependentWatchdog,
    prelude::*,
    rcc::{self, rec::FdcanClkSel},
};

use core::num::{NonZeroU16, NonZeroU8};
use embedded_graphics::pixelcolor::Rgb666;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    prelude::*,
    text::{Baseline, Text}
};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::delay::DelayMs;
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    interrupt::{InterruptLine, Interrupts},
};
use rtic_monotonics::{systick::*, Monotonic};
use mipidsi::Builder;
use display_interface_parallel_gpio::{Generic16BitBus, PGpio16BitInterface};

pub type Duration =
    <rtic_monotonics::systick::Systick as rtic_monotonics::Monotonic>::Duration;

pub struct FakeDelay {}

impl DelayUs<u32> for FakeDelay {
    fn delay_us(&mut self, t: u32) {
        let start = Systick::now();
        let dur = Duration::micros(t.into());
        while Systick::now() - start < dur {}
    }
}

impl DelayMs<u32> for FakeDelay {
    fn delay_ms(&mut self, t: u32) {
        let start = Systick::now();
        let dur = Duration::millis(t.into());
        while Systick::now() - start < dur {}
    }
}

pub fn init(cx: init::Context) -> (Shared, Local) {
    defmt::info!("init");

    // Setup and start independent watchdog.
    // Initialisation must complete before the watchdog triggers
    let watchdog = {
        let mut wd = IndependentWatchdog::new(cx.device.IWDG1);
        //wd.start(100_u32.millis());
        wd
    };

    // configure power domain
    let pwr = cx
        .device
        .PWR
        .constrain()
        .backup_regulator()
        .smps()
        .vos0(&cx.device.SYSCFG)
        .freeze();

    // RCC
    let rcc = cx.device.RCC.constrain();
    let ccdr = rcc
        .sysclk(480.MHz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_q_ck(200.MHz())
        .freeze(pwr, &cx.device.SYSCFG);

    // Monotonics
    Systick::start(
        cx.core.SYST,
        ccdr.clocks.sysclk().to_Hz(),
        rtic_monotonics::create_systick_token!(),
    );

    // GPIO
    let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = cx.device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = cx.device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = cx.device.GPIOE.split(ccdr.peripheral.GPIOE);

    // Status LEDs
    let led_ok = gpiob.pb10.into_push_pull_output().erase();
    let led_warn = gpiob.pb11.into_push_pull_output().erase();
    let led_error = gpiob.pb12.into_push_pull_output().erase();

    // CAN
    let can = {
        let tx = gpioa.pa12.into_alternate().speed(Speed::VeryHigh);
        let rx = gpioa.pa11.into_alternate().speed(Speed::VeryHigh);
        let fdcan_prec = ccdr.peripheral.FDCAN.kernel_clk_mux(FdcanClkSel::Pll1Q);
        let mut can = cx.device.FDCAN1.fdcan(tx, rx, fdcan_prec);

        // throw error rather than trying to handle unexpected bus behaviour
        can.set_protocol_exception_handling(false);

        // k-clock 32MHz, bit rate 500kbit/s, sample point 87.5%
        can.set_nominal_bit_timing(NominalBitTiming {
            prescaler: NonZeroU16::new(4).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        });
        // k-clock 32MHz, bit rate 500kbit/s, sample point 87.5%
        can.set_data_bit_timing(DataBitTiming {
            prescaler: NonZeroU8::new(4).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
            transceiver_delay_compensation: true,
        });

        can.enable_interrupt_line(InterruptLine::_0, true);
        can.enable_interrupt_line(InterruptLine::_1, true);
        can.enable_interrupts(Interrupts::RX_FIFO0_NEW_MSG | Interrupts::RX_FIFO1_NEW_MSG);

        can.into_external_loopback()
    };

    let dc = gpiob.pb4.into_push_pull_output();
    let mut res = gpioc.pc6.into_push_pull_output_in_state(stm32h7xx_hal::gpio::PinState::High);

    res.set_high();

    let mut delay = FakeDelay {};

    // Define the pins used for the parallel interface as digital outputs
    let lcd_d0 = gpiod.pd15.into_push_pull_output();
    let lcd_d1 = gpiod.pd14.into_push_pull_output();
    let lcd_d2 = gpiod.pd13.into_push_pull_output();
    let lcd_d3 = gpiod.pd12.into_push_pull_output();
    let lcd_d4 = gpiod.pd11.into_push_pull_output();
    let lcd_d5 = gpioe.pe2.into_push_pull_output();
    let lcd_d6 = gpiob.pb2.into_push_pull_output();
    let lcd_d7 = gpiob.pb6.into_push_pull_output();
    let lcd_d8 = gpioa.pa15.into_push_pull_output();
    let lcd_d9 = gpiob.pb8.into_push_pull_output();
    let lcd_d10 = gpiob.pb9.into_push_pull_output();
    let lcd_d11 = gpioe.pe7.into_push_pull_output();
    let lcd_d12 = gpioe.pe10.into_push_pull_output();
    let lcd_d13 = gpioe.pe12.into_push_pull_output();
    let lcd_d14 = gpioe.pe14.into_push_pull_output();
    let lcd_d15 = gpioe.pe15.into_push_pull_output();

    // Define the parallel bus with the previously defined parallel port pins
    let bus = Generic16BitBus::new((
        lcd_d0, lcd_d1, lcd_d2, lcd_d3, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
        lcd_d8, lcd_d9, lcd_d10, lcd_d11, lcd_d12, lcd_d13, lcd_d14, lcd_d15
    ));
    let wr = gpiob.pb15.into_push_pull_output_in_state(stm32h7xx_hal::gpio::PinState::High);

    // Define the display interface from a generic 8 bit bus, a Data/Command select pin and a write enable pin
    let di = PGpio16BitInterface::new(bus, dc, wr);
    
    let builder = Builder::ili9486_rgb666(di);
    defmt::info!("Build finished.");
    let mut display = builder.init(&mut delay, Some(res)).unwrap();
    defmt::info!("Display finished.");
    display.set_orientation(mipidsi::Orientation::LandscapeInverted(true)).unwrap();

    // draw things
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb666::BLUE)
        .build();

    let text_style2 = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb666::RED)
        .build();

    Text::with_baseline("Hello World!", Point::new(10, 100), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::new("Hello World again!", Point::new(210, 300), text_style2)
        .draw(&mut display)
        .unwrap();

    display.set_pixels(300, 100, 350, 150, core::iter::repeat(Rgb666::YELLOW).take(50*50)).unwrap();

    // display.clear(Rgb666::RED).unwrap();
    
    watchdog::spawn().ok();
    update_display::spawn().ok();
    
    defmt::info!("Initialisation finished.");

    (
        Shared {
            can,
        },
        Local {
            watchdog,
            led_ok,
            led_warn,
            led_error,
            display
        },
    )
}
