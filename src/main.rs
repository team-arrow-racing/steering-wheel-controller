//! Steering Wheel
//!
//! # Task priority assignment
//!
//! It would be more idomatic to have these assigned in a enum or some constants
//! but RTIC doesn't yet support variables (static or otherwise) in task
//! definitions.
//!
//! | Priority | Use |
//! | --- | --- |
//! | 0 | `idle` task and background tasks. |
//! | 1 (default) | General and asychronous tasks. |
//! | 2 | Synchronous comms tasks. |
//! | 3 | System critical tasks. |

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use dwt_systick_monotonic::{fugit, DwtSystick};
use solar_car::{com, device};

use stm32l4xx_hal::{
    can::Can,
    device::CAN1,
    flash::FlashExt,
    gpio::{
        Alternate, Edge, ExtiPin, Input, Output, PinExt, PullUp, PushPull,
        PA11, PA12, PA14, PA8, PB13, PB5,
    },
    prelude::*,
    stm32,
    watchdog::IndependentWatchdog,
};

use cortex_m::peripheral::NVIC;

const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {

    use bxcan::{filter::Mask32, Interrupts};

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    #[shared]
    struct Shared {
        can: bxcan::Can<
            Can<
                CAN1,
                (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>),
            >,
        >,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,
        left_indicator_btn: PA8<Input<PullUp>>,
        right_indicator_btn: PB5<Input<PullUp>>, // TODO figure out which pins
        horn_btn: PA14<Input<PullUp>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::trace!("task: init");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc
            .cfgr
            .sysclk(80.MHz())
            .pclk1(80.MHz())
            .pclk2(80.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().to_Hz(),
        );

        // configure status led
        let status_led = gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // TODO: 8 buttons in total
        // figure out a way to have all these in an array or map {button: pin}
        let left_indicator_btn = {
            let mut btn = gpioa
                .pa8
                .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(stm32::Interrupt::EXTI9_5);
            }

            btn
        };

        let right_indicator_btn = {
            let mut btn = gpiob
                .pb5
                .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(stm32::Interrupt::EXTI9_5);
            }

            btn
        };

        let horn_btn = {
            let mut btn = gpioa
                .pa14
                .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(stm32::Interrupt::EXTI9_5);
            }

            btn
        };

        // configure can bus
        let can = {
            let rx = gpioa.pa11.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );
            let tx = gpioa.pa12.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );

            let can = bxcan::Can::builder(Can::new(
                &mut rcc.apb1r1,
                cx.device.CAN1,
                (tx, rx),
            ))
            .set_bit_timing(0x001c_0009); // 500kbit/s

            let mut can = can.enable();

            // configure filters
            can.modify_filters().enable_bank(0, Mask32::accept_all());

            // configure interrupts
            can.enable_interrupts(
                Interrupts::TRANSMIT_MAILBOX_EMPTY
                    | Interrupts::FIFO0_MESSAGE_PENDING
                    | Interrupts::FIFO1_MESSAGE_PENDING,
            );
            nb::block!(can.enable_non_blocking()).unwrap();

            // broadcast startup message.
            can.transmit(&com::startup::message(DEVICE)).unwrap();

            can
        };

        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        run::spawn().unwrap();

        (
            Shared { can },
            Local {
                watchdog,
                status_led,
                left_indicator_btn,
                right_indicator_btn,
                horn_btn,
            },
            init::Monotonics(mono),
        )
    }

    // TODO each task needs rising or falling edge as parameter

    #[task(priority = 1, local = [watchdog])]
    fn run(cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    /// Triggers on interrupt event.
    #[task(priority = 1, binds = EXTI9_5, local = [left_indicator_btn, right_indicator_btn, horn_btn])]
    fn exti9_5_pending(cx: exti9_5_pending::Context) {
        defmt::trace!("task: exti9_5 pending");

        let left_indicator_btn = cx.local.left_indicator_btn;
        let right_indicator_btn = cx.local.right_indicator_btn;
        let horn_btn = cx.local.horn_btn;

        if left_indicator_btn.check_interrupt() {
            defmt::trace!(
                "Interrupt triggered on {:?}",
                left_indicator_btn.pin_id()
            );
            left_indicator_btn.clear_interrupt_pending_bit();
            button_indicator_left_handler::spawn().unwrap();
        }

        if right_indicator_btn.check_interrupt() {
            defmt::trace!(
                "Interrupt triggered on {:?}",
                right_indicator_btn.pin_id()
            );
            right_indicator_btn.clear_interrupt_pending_bit();
            button_indicator_right_handler::spawn().unwrap();
        }

        if horn_btn.check_interrupt() {
            defmt::trace!("Interrupt triggered on {:?}", horn_btn.pin_id());
            horn_btn.clear_interrupt_pending_bit();
            button_horn_handler::spawn().unwrap();
        }
    }

    #[task(priority = 2, shared = [can])]
    fn button_indicator_left_handler(
        mut cx: button_indicator_left_handler::Context,
    ) {
        defmt::trace!("task: can send left indicator frame");
        let light_frame = com::lighting::message(
            DEVICE,
            com::lighting::LampsState::INDICATOR_LEFT.bits(),
        );

        cx.shared.can.lock(|can| {
            let _ = can.transmit(&light_frame);
        });
    }

    #[task(priority = 2, shared = [can])]
    fn button_indicator_right_handler(
        mut cx: button_indicator_right_handler::Context,
    ) {
        defmt::trace!("task: can send right indicator frame");
        let light_frame = com::lighting::message(
            DEVICE,
            com::lighting::LampsState::INDICATOR_RIGHT.bits(),
        );

        cx.shared.can.lock(|can| {
            let _ = can.transmit(&light_frame);
        });
    }

    #[task(priority = 2, shared = [can])]
    fn button_horn_handler(mut cx: button_horn_handler::Context) {
        cx.shared.can.lock(|can| {});
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::trace!("task: idle");

        loop {
            cortex_m::asm::nop();
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// Show a millisecond timestamp next to debug output.
// Unit conversion isn't required because ticks = milliseconds for our case.
defmt::timestamp!("time={=u64}ms", {
    app::monotonics::MonoTimer::now()
        .duration_since_epoch()
        .to_millis()
});
