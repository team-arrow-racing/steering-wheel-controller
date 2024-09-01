use crate::app::{init, watchdog, Local, Shared};
use crate::hal::{
    delay::{Delay, DelayFromCountDownTimer},
    gpio::Speed,
    independent_watchdog::IndependentWatchdog,
    prelude::*,
    rcc::{self, rec::FdcanClkSel},
    spi
};

use core::borrow::BorrowMut;
use core::num::{NonZeroU16, NonZeroU8};
use display_interface::DisplayError;
use embedded_graphics::pixelcolor::{Gray4, Rgb565};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    prelude::*,
    text::{Baseline, Text}
};
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    interrupt::{InterruptLine, Interrupts},
};
use ili9341::Ili9341;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use ssd1320::buffered_graphics::BufferedSsd1320z2;
use rtic_monotonics::{systick::*, Monotonic};
use ili9486::{Command, ILI9486};
use display_interface_spi::SPIInterface;

pub fn init(cx: init::Context) -> (Shared, Local) {
    defmt::info!("init");

    // Setup and start independent watchdog.
    // Initialisation must complete before the watchdog triggers
    let watchdog = {
        let mut wd = IndependentWatchdog::new(cx.device.IWDG1);
        wd.start(100_u32.millis());
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

    // LCD Display
    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb8.into_alternate_open_drain();
    let sda = gpiob.pb9.into_alternate_open_drain();

    let i2c = cx.device.I2C1.i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    // display.init().unwrap();

    defmt::info!("Display finished.");

    // Second LCD
    let sck = gpioc.pc10.into_alternate();
    let miso = gpioc.pc11.into_alternate();
    let mosi = gpioc.pc12.into_alternate();

    let dc = gpiob.pb4.into_push_pull_output();
    let res = gpiob.pb13.into_push_pull_output();
    let cs = gpiob.pb14.into_push_pull_output();

    // Initialise the SPI peripheral.
    let spi = cx.device.SPI3.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        8.MHz(),
        ccdr.peripheral.SPI3,
        &ccdr.clocks,
    );

    let timer = cx.device
        .TIM2
        .timer(1.kHz(), ccdr.peripheral.TIM2, &ccdr.clocks);
    let mut delay = DelayFromCountDownTimer::new(timer);
    let iface = SPIInterface::new(spi, dc, cs);

    // let mut lcd_driver = ILI9486::new(
    //     &mut delay,
    //     ili9486::color::PixelFormat::Rgb565,
    //     iface,
    //     ili9486::io::shim::OutputOnlyIoPin::new(res),
    // )
    // .unwrap();

    let mut lcd = Ili9341::new(
        iface,
        res,
        &mut delay,
        ili9341::Orientation::Portrait,
        ili9341::DisplaySize320x480,
    )
    .unwrap();
    let area = Rectangle::new(Point::new(100,100), Size::new(100, 200));
    lcd.fill_solid(&area, Rgb565::RED).unwrap();

    // draw things
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::BLUE)
        .build();

    Text::with_baseline("Hello World!", Point::new(50, 50), text_style, Baseline::Top)
        .draw(&mut lcd)
        .unwrap();
    
    watchdog::spawn().ok();

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
            // display
        },
    )
}
