use crate::app::{init, watchdog, update_display, Local, Shared};
use crate::hal::{
    gpio::Speed,
    independent_watchdog::IndependentWatchdog,
    prelude::*,
    rcc::{self, rec::FdcanClkSel},
};

use core::num::{NonZeroU16, NonZeroU8};
use fdcan::{
    config::{DataBitTiming, NominalBitTiming},
    interrupt::{InterruptLine, Interrupts},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use rtic_monotonics::systick::*;

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
    display.init().unwrap();

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
