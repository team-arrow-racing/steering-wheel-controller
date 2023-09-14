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
use solar_car::{
    com, device, j1939,
    j1939::pgn::{Number, Pgn},
};

use bxcan::{filter::Mask32, Frame, Id, Interrupts};

use stm32l4xx_hal::{
    can::Can,
    device::CAN1,
    delay::DelayCM,
    flash::FlashExt,
    gpio::{
        Alternate, Edge, ExtiPin, Input, Output, PullUp, PushPull,
        PA0, // AI2
        PA1, // AI1
        PA4, // PWM6
        PA5, // PWM5
        PA6, // PWM8
        PA7, // PWM7
        PA8, // DI5
        PA9, // DI6
        PA10, // DI7
        PA11, // CAN RX
        PA12, // CAN TX
        PA14, // Horn btn (?)
        PA15, // DI8
        PB5, // STATUS_LED
        PB6, // INPUT_PB
        PC0, // PWM2
        PC1, // PWM1
        PC2, // PWM4
        PC3, // PWM3
        PC6, // DI1
        PC7, // DI2
        PC8, // DI3
        PC9, // DI4
        PC10, // DI9
        PC11, // DI10
        PC12, // DI11
        PD2, // DI12

    },
    prelude::*,
    stm32::Interrupt,
    watchdog::IndependentWatchdog,
};

use cortex_m::peripheral::NVIC;
mod lcd;

use lcd::LCD;

const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;
pub const WARNING_CODES: &str = "ABCDEFG";

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(
        buffer: &'a mut [u8],
        args: fmt::Arguments,
    ) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}

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
        speed: u8,
        battery: u8,
        temperature: u8,
        left_indicator: bool,
        right_indicator: bool,
        warnings: [u8; 6]
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        led_status: PB5<Output<PushPull>>,
        btn_indicator_left: PC6<Input<PullUp>>,
        btn_indicator_right: PC7<Input<PullUp>>, // TODO figure out which pins
        btn_horn: PA14<Input<PullUp>>,
        lcd_disp: LCD
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
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb2);

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
        let led_status = gpiob
            .pb5
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // TODO: 8 buttons in total
        // figure out a way to have all these in an array or map {button: pin}
        let btn_indicator_left = {
            let mut btn = gpioc
                .pc6
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_indicator_right = {
            let mut btn = gpioc
                .pc7
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_horn = {
            let mut btn = gpioa
                .pa14
                .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        // TODO figure out which buttons are which pins
        let btn_drive = {
            let mut btn = gpioc
                .pc8
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_neutral = {
            let mut btn = gpioc
                .pc9
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_reverse = {
            let mut btn = gpioc
                .pc10
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_cruise = {
            let mut btn = gpioc
                .pc11
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let mut lcd_disp = {
            let rs = gpioc
                .pc2
                .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
            let en = gpioc
                .pc3
                .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
            let d4 = gpioa
                .pa6
                .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
            let d5 = gpioa
                .pa7
                .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
            let d6 = gpioa
                .pa4
                .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
            let d7 = gpioa
                .pa5
                .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
            // let delay = Delay::new(cx.core.SYST, clocks);
            let delay_cm = DelayCM::new(clocks);

            let mut disp = LCD::new(
                rs, en, d4, d5, d6, d7, delay_cm,
            );

            disp
        };

        lcd_disp.init();

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
        heartbeat::spawn().unwrap();
        update_display::spawn().unwrap();

        (
            Shared { 
                can,
                speed: 100,
                battery: 12, 
                temperature: 65,
                left_indicator: false,
                right_indicator: false,
                warnings: [0, 0, 0, 0, 0, 0] 
            },
            Local {
                watchdog,
                led_status,
                btn_indicator_left,
                btn_indicator_right,
                btn_horn,
                lcd_disp
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

    #[task(local = [led_status], shared = [can])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.led_status.toggle();

        if cx.local.led_status.is_set_low() {
            cx.shared.can.lock(|can| {
                let _ = can.transmit(&com::heartbeat::message(DEVICE));
            });
        }

        // repeat every second
        heartbeat::spawn_after(500.millis().into()).unwrap();
    }

    /// Triggers on interrupt event.
    #[task(priority = 1, binds = EXTI9_5, local = [btn_indicator_left, btn_indicator_right, btn_horn])]
    fn exti9_5_pending(cx: exti9_5_pending::Context) {
        defmt::trace!("task: exti9_5 pending");

        let left_indicator_btn = cx.local.btn_indicator_left;
        let right_indicator_btn = cx.local.btn_indicator_right;
        let horn_btn = cx.local.btn_horn;

        if left_indicator_btn.check_interrupt() {
            left_indicator_btn.clear_interrupt_pending_bit();
            button_indicator_left_handler::spawn(left_indicator_btn.is_high())
                .unwrap();
        }

        if right_indicator_btn.check_interrupt() {
            right_indicator_btn.clear_interrupt_pending_bit();
            button_indicator_right_handler::spawn(
                right_indicator_btn.is_high(),
            )
            .unwrap();
        }

        if horn_btn.check_interrupt() {
            horn_btn.clear_interrupt_pending_bit();
            button_horn_handler::spawn(horn_btn.is_high()).unwrap();
        }
    }

    /// Handle left indictor button state change
    #[task(priority = 2, shared = [can])]
    fn button_indicator_left_handler(
        mut cx: button_indicator_left_handler::Context,
        state: bool,
    ) {
        defmt::trace!("task: can send left indicator frame");
        let light_frame = com::lighting::message(
            DEVICE,
            com::lighting::LampsState::INDICATOR_LEFT.bits(),
        );
        // TODO change l and r to show on display

        cx.shared.can.lock(|can| {
            let _ = can.transmit(&light_frame);
        });
    }

    /// Handle left indicator button state change
    #[task(priority = 2, shared = [can])]
    fn button_indicator_right_handler(
        mut cx: button_indicator_right_handler::Context,
        state: bool,
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

    /// Handle horn button state change
    #[task(priority = 2, shared = [can])]
    fn button_horn_handler(mut cx: button_horn_handler::Context, state: bool) {
        cx.shared.can.lock(|can| {});
    }

    /// Handle updating of speed
    #[task(shared = [speed, battery, temperature, left_indicator, right_indicator, warnings], local=[lcd_disp])]
    fn update_display(cx: update_display::Context) {
        let speed = cx.shared.speed;
        let battery = cx.shared.battery;
        let temp = cx.shared.temperature;
        let l_ind = cx.shared.left_indicator;
        let r_ind = cx.shared.right_indicator;
        let warns = cx.shared.warnings;
       

        let lcd = cx.local.lcd_disp;

        (speed, battery, temp, l_ind, r_ind, warns).lock(|speed, battery, temp, l_ind, r_ind, warns| {
            // Display look should as following
            // |L_100_100_100_R|
            // |    ABCDEFG    |
            // Start by clearing everything
            lcd.clear_display();
            // Set indicator status
            if *l_ind {
                lcd.set_position(0, 0);
                lcd.send_string("L");
            }

            if *r_ind {
                lcd.set_position(15, 0);
                lcd.send_string("R");
            }

            let mut data_buf = [0; 3];

            let mut vehicle_data = crate::write_to::show(
                &mut data_buf,
                format_args!(
                    "{}",
                    battery
                ),
            )
            .unwrap();

            lcd.set_position(2, 0);
            lcd.send_string(vehicle_data);

            vehicle_data = crate::write_to::show(
                &mut data_buf,
                format_args!(
                    "{}",
                    speed
                ),
            )
            .unwrap();

            lcd.set_position(6, 0);
            lcd.send_string(vehicle_data);

            vehicle_data = crate::write_to::show(
                &mut data_buf,
                format_args!(
                    "{}",
                    temp
                ),
            )
            .unwrap();

            lcd.set_position(10, 0);
            lcd.send_string(vehicle_data);
            
            // Warnings
            for i in 0..warns.len() {
                lcd.set_position((4 + i).try_into().unwrap(), 1);
                if warns[i] == 1 {
                    lcd.send_data(WARNING_CODES.chars().nth(i).unwrap().try_into().unwrap());
                } else {
                    lcd.send_string("_");
                }
            }

            update_display::spawn_after(Duration::millis(10)).unwrap();

        });
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(_: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");

        can_receive::spawn().unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(_: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");

        can_receive::spawn().unwrap();
    }

    #[task(priority = 2, shared = [can, speed, battery, temperature, left_indicator, right_indicator, warnings])]
    fn can_receive(mut cx: can_receive::Context) {
        defmt::trace!("task: can receive");

        cx.shared.can.lock(|can| loop {
            let frame = match can.receive() {
                Ok(frame) => frame,
                Err(nb::Error::WouldBlock) => break, // done
                Err(nb::Error::Other(_)) => continue, // go to next frame
            };

            let id = match frame.id() {
                Id::Standard(_) => {
                    continue; // go to next frame
                }
                Id::Extended(id) => id,
            };

            let id: j1939::ExtendedId = id.into();

            match id.pgn {
                Pgn::Destination(pgn) => match pgn {
                    com::wavesculptor::PGN_BATTERY_MESSAGE => {
                        cx.shared.battery.lock(|battery| {
                            if let Some(data) = frame.data() {
                                // TODO might need to handle multiple bits
                                *battery = data[0];
                            }
                        });
                    },
                    com::wavesculptor::PGN_SPEED_MESSAGE => {
                        cx.shared.speed.lock(|speed| {
                            if let Some(data) = frame.data() {
                                *speed = data[0];
                            }
                        });
                    },
                    com::wavesculptor::PGN_TEMPERATURE_MESSAGE => {
                        cx.shared.temperature.lock(|temp| {
                            if let Some(data) = frame.data() {
                                *temp = data[0];
                            }
                        });
                    },
                    _ => {
                        defmt::debug!("whut happun")
                    }
                },
                _ => {} // ignore broadcast messages
            }
        });
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
