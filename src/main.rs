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

use timed_debouncer::Debouncer;
use dwt_systick_monotonic::{fugit, DwtSystick};
use solar_car::{
    com::{self, 
        wavesculptor::{
            self,
            DriverModes,
        }}, 
    device, 
    j1939,
    j1939::pgn::{Number, Pgn},
};

use bxcan::{filter::Mask32, Frame, Id, Interrupts, StandardId};
use numtoa::NumToA;
use phln::wavesculptor::{ErrorFlags, WaveSculptor};

use stm32l4xx_hal::{
    adc::{DmaMode, SampleTime, Sequence, ADC},
    can::Can,
    delay::DelayCM,
    device::CAN1,
    flash::FlashExt,
    gpio::{
        Analog,
        Alternate,
        Edge,
        ExtiPin,
        Input,
        Output,
        PullUp,
        PullDown,
        PushPull,
        PA0,  // AI2
        PA1,  // AI1
        PA10, // DI7
        PA11, // CAN RX
        PA12, // CAN TX
        PA15, // DI8
        PA4,  // PWM6
        PA5,  // PWM5
        PA6,  // PWM8
        PA7,  // PWM7
        PA8,  // DI5
        PA9,  // DI6
        PB5,  // STATUS_LED
        PB6,  // INPUT_PB
        PC0,  // PWM2
        PC1,  // PWM1
        PC10, // DI9 Horn button
        PC11, // DI10 Cruise button
        PC12, // DI11
        PC2,  // PWM4
        PC3,  // PWM3
        PC6,  // DI1 INDICATOR LEFT
        PC7,  // DI2 
        PC8,  // DI3 INDICATOR RIGHT
        PC9,  // DI4 CONTACTOR SWITCH
        PD2,  // DI12
    },
    prelude::*,
    stm32::Interrupt,
    watchdog::IndependentWatchdog,
};

use cortex_m::peripheral::NVIC;
mod lcd;

use lcd::LCD;

pub struct LcdData {
    speed: f32,
    battery: f32,
    temperature: f32,
    left_indicator: bool, // What shows on the LCD screen, meant to be toggled while input is true
    right_indicator: bool,
    mode: DriverModes, // Drive, Neutral, or Reverse
    cruise: bool, 
    warnings: [u8; 8],
    contactors: bool, // Contactors enabled or not
}

pub struct InputButtons9_5 {
    btn_indicator_left: PC6<Input<PullUp>>,
    btn_indicator_right: PC8<Input<PullUp>>, // TODO figure out which pins
}

pub struct InputButtons15_10 {
    btn_horn: PC10<Input<PullUp>>,
    btn_cruise: PC11<Input<PullUp>>,
}

const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;
pub const WARNING_CODES: &str = "ABCDEFG";

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {

    use bxcan::{filter::Mask32, Interrupts};

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;
    pub type Can1Pins =
        (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>);

    #[shared]
    struct Shared {
        can: bxcan::Can<Can<CAN1, Can1Pins>>,
        lcd_data: LcdData,
        input_left_indicator: bool,
        input_right_indicator: bool,
        last_ind_sent: Instant,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        led_status: PB5<Output<PushPull>>,
        input_buttons_9_5: InputButtons9_5,
        input_buttons_15_10: InputButtons15_10,
        lcd_disp: LCD,
        adc: ADC,
        driver_pot: PA0<Analog>,
        enable_contactor_switch: PC9<Input<PullUp>>,
        left_ind_light: PC1<Output<PushPull>>,
        right_ind_light: PC0<Output<PushPull>>,
        ws22: WaveSculptor
    }

    static MAXIMUM_DURATION: Duration = Duration::millis(2000);

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
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_indicator_right = {
            let mut btn = gpioc
                .pc8
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        let btn_horn = {
            let mut btn = gpioc
                .pc10
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI15_10);
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
                NVIC::unmask(Interrupt::EXTI15_10);
            }

            btn
        };

        let enable_contactor_switch = {
            let mut btn = gpioc
                .pc9
                .into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);

            btn.make_interrupt_source(&mut cx.device.SYSCFG, &mut rcc.apb2);
            btn.enable_interrupt(&mut cx.device.EXTI);
            btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

            unsafe {
                NVIC::unmask(Interrupt::EXTI9_5);
            }

            btn
        };

        // TODO button to send 0x30 data with 0x505 id

        let input_buttons_9_5 = InputButtons9_5 {
            btn_indicator_left,
            btn_indicator_right
        };

        let input_buttons_15_10 = InputButtons15_10 {
            btn_horn,
            btn_cruise,
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

            let disp = LCD::new(rs, en, d4, d5, d6, d7, delay_cm);

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
                    // | Interrupts::FIFO1_MESSAGE_PENDING,
            );
            nb::block!(can.enable_non_blocking()).unwrap();

            // broadcast startup message.
            nb::block!(can.transmit(&com::startup::message(DEVICE))).unwrap();

            can
        };

        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        let lcd_data = LcdData {
            speed: 0f32,
            battery: 0f32,
            temperature: 0f32,
            left_indicator: false,
            right_indicator: false,
            mode: DriverModes::Neutral,
            cruise: false, // torque mode by default
            warnings: [0, 0, 0, 0, 0, 0, 0, 0],
            contactors: enable_contactor_switch.is_low(),
        };

        let mut delay = DelayCM::new(clocks);
        let adc = ADC::new(
            cx.device.ADC1,
            cx.device.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );
        let driver_pot =
            gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        let left_ind_light = gpioc
            .pc1
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

        let right_ind_light = gpioc
            .pc0
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

        let last_ind_sent = Instant::from_ticks(0);

        let ws22 = WaveSculptor::new(phln::wavesculptor::ID_BASE);

        run::spawn().unwrap();
        heartbeat::spawn_after(Duration::millis(500)).unwrap();
        update_display::spawn().unwrap();
        read_driver_pot::spawn().unwrap();
        toggle_left_indicator::spawn().unwrap();
        toggle_right_indicator::spawn().unwrap();

        (
            Shared {
                can,
                lcd_data,
                input_left_indicator: input_buttons_9_5.btn_indicator_left.is_high(),
                input_right_indicator: input_buttons_9_5.btn_indicator_right.is_high(),
                last_ind_sent
            },
            Local {
                watchdog,
                led_status,
                input_buttons_9_5,
                input_buttons_15_10,
                lcd_disp,
                adc,
                driver_pot,
                enable_contactor_switch,
                left_ind_light,
                right_ind_light,
                ws22
            },
            init::Monotonics(mono),
        )
    }

    // TODO each task needs rising or falling edge as parameter

    #[task(priority = 1, shared = [last_ind_sent, input_left_indicator, input_right_indicator], local = [watchdog, left_ind_light, right_ind_light])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        let time = monotonics::MonoTimer::now();

        cx.shared.last_ind_sent.lock(|last| {
            if time.checked_duration_since(*last).unwrap() > MAXIMUM_DURATION {
                cx.shared.input_left_indicator.lock(|l_ind| {
                    let _ =  button_indicator_left_handler::spawn(*l_ind);
                });

                cx.shared.input_right_indicator.lock(|r_ind| {
                    let _ =  button_indicator_right_handler::spawn(*r_ind);
                });
            }
        });

        let l_light = cx.local.left_ind_light;
        let r_light = cx.local.right_ind_light;
        let on: bool = (time.duration_since_epoch().to_millis() % 1000) > 500;

        cx.shared.input_left_indicator.lock(|l_ind| {
            l_light.set_state(PinState::from(
                *l_ind && on
            ));
            defmt::trace!("l light {}", l_light.is_set_high());
        });

        cx.shared.input_right_indicator.lock(|r_ind| {
            r_light.set_state(PinState::from(
                *r_ind && on
            ));
            defmt::trace!("r light {}", r_light.is_set_high());
        });
        
        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(local = [led_status], shared = [can])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.led_status.toggle();

        if cx.local.led_status.is_set_low() {
            cx.shared.can.lock(|can| {
                nb::block!(can.transmit(&com::heartbeat::message(DEVICE))).unwrap();
            });
        }

        // repeat every second
        heartbeat::spawn_after(500.millis().into()).unwrap();
    }

    /// Triggers on interrupt event.
    #[task(priority = 2, binds = EXTI9_5, local = [input_buttons_9_5, enable_contactor_switch])]
    fn exti9_5_pending(cx: exti9_5_pending::Context) {
        defmt::trace!("task: exti9_5 pending");

        let btn_ind_left = &mut cx.local.input_buttons_9_5.btn_indicator_left;
        let btn_ind_right = &mut cx.local.input_buttons_9_5.btn_indicator_right;
        let enable_contactor_switch = cx.local.enable_contactor_switch;

        if btn_ind_left.check_interrupt() {
            defmt::debug!("left ind {}", btn_ind_left.is_high());
            btn_ind_left.clear_interrupt_pending_bit();
            button_indicator_left_handler::spawn(btn_ind_left.is_high())
                .unwrap();
        }

        if btn_ind_right.check_interrupt() {
            defmt::debug!("right ind {}", btn_ind_right.is_high());
            btn_ind_right.clear_interrupt_pending_bit();
            button_indicator_right_handler::spawn(btn_ind_right.is_high())
                .unwrap();
        }

        if enable_contactor_switch.check_interrupt() {
            enable_contactor_switch.clear_interrupt_pending_bit();
            toggle_contactors::spawn().unwrap();
        }
    }

    #[task(priority = 2, binds = EXTI15_10, local = [input_buttons_15_10])]
    fn exti15_10_pending(cx: exti15_10_pending::Context) {
        let btn_horn = &mut cx.local.input_buttons_15_10.btn_horn;
        let btn_cruise = &mut cx.local.input_buttons_15_10.btn_cruise;

        if btn_horn.check_interrupt() {
            btn_horn.clear_interrupt_pending_bit();
            defmt::debug!("horn pressed {}", btn_horn.is_high());
            let frame = com::horn::horn_message(DEVICE, btn_horn.is_high() as u8);
            send_can_frame::spawn(frame).unwrap();
        }

        if btn_cruise.check_interrupt() {
            btn_cruise.clear_interrupt_pending_bit();
            button_cruise_toggle_handler::spawn().unwrap();
        }
    }

    #[task(priority = 2, shared = [lcd_data])]
    fn toggle_contactors(mut cx: toggle_contactors::Context) {
        cx.shared.lcd_data.lock(|lcd_data| {
            lcd_data.contactors = !lcd_data.contactors;
            let frame = com::array::enable_contactors_message(DEVICE, lcd_data.contactors);
            send_can_frame::spawn(frame).unwrap();
        });
    }

    /// Handle left indictor button state change
    #[task(priority = 2, shared = [can, last_ind_sent, input_left_indicator])]
    fn button_indicator_left_handler(
        mut cx: button_indicator_left_handler::Context,
        state: bool,
    ) {
        defmt::trace!("task: can send left indicator frame");
        let light_frame = com::lighting::message(
            DEVICE,
            com::lighting::LampsState::INDICATOR_LEFT,
            state as u8
        );

        cx.shared.input_left_indicator.lock(|l_input| {
            *l_input = state;
        });

        cx.shared.can.lock(|can| {
            nb::block!(can.transmit(&light_frame)).unwrap();
        });
        
        cx.shared.last_ind_sent.lock(|last| {
            *last = monotonics::MonoTimer::now();
        });
    }

    #[task(priority = 1, shared = [lcd_data])]
    fn button_driver_mode_handler(mut cx: button_driver_mode_handler::Context, mode: DriverModes) {
        cx.shared.lcd_data.lock(|lcd_data| {
            lcd_data.mode = mode;
            let frame = wavesculptor::driver_mode_message(DEVICE, mode);
            send_can_frame::spawn(frame).unwrap();
        });
    }

    #[task(priority = 1, shared = [lcd_data])]
    fn button_cruise_toggle_handler(mut cx: button_cruise_toggle_handler::Context) {
        cx.shared.lcd_data.lock(|lcd_data| {
            lcd_data.cruise = !lcd_data.cruise;
            defmt::debug!("cruise is now {}", lcd_data.cruise);
            let frame = wavesculptor::control_type_message(DEVICE, lcd_data.cruise);
            send_can_frame::spawn(frame).unwrap();
        });
    }

    #[task(priority = 1, shared = [input_left_indicator, lcd_data])]
    fn toggle_left_indicator(mut cx: toggle_left_indicator::Context) {
        cx.shared.input_left_indicator.lock(|l_input| {
            cx.shared.lcd_data.lock(|lcd_data| {
                lcd_data.left_indicator = *l_input;
                // TODO handle actual light output
            });
        });

        toggle_left_indicator::spawn_after(Duration::millis(100)).unwrap();
    }

    /// Handle right indicator button state change
    #[task(priority = 1, shared = [can, input_right_indicator, last_ind_sent])]
    fn button_indicator_right_handler(
        mut cx: button_indicator_right_handler::Context,
        state: bool,
    ) {
        defmt::trace!("task: can send right indicator frame");
        let light_frame = com::lighting::message(
            DEVICE,
            com::lighting::LampsState::INDICATOR_RIGHT,
            state as u8
        );

        cx.shared.input_right_indicator.lock(|r_input| {
            *r_input = state;
        });

        cx.shared.can.lock(|can| {
            nb::block!(can.transmit(&light_frame)).unwrap();
        });

        cx.shared.last_ind_sent.lock(|last| {
            *last = monotonics::MonoTimer::now();
        });
    }

    #[task(priority = 1, shared = [input_right_indicator, lcd_data])]
    fn toggle_right_indicator(mut cx: toggle_right_indicator::Context) {
        cx.shared.input_right_indicator.lock(|r_input| {
            cx.shared.lcd_data.lock(|lcd_data| {
                lcd_data.right_indicator = *r_input;
                // TODO handle actual light output
            });
        });

        toggle_right_indicator::spawn_after(Duration::millis(100)).unwrap();
    }

    #[task(priority = 1, shared = [can])]
    fn send_can_frame(mut cx: send_can_frame::Context, frame: Frame) {
        cx.shared.can.lock(|can| {
            nb::block!(can.transmit(&frame)).unwrap();
        });
    }

    #[task(shared = [lcd_data], local = [adc, driver_pot])]
    fn read_driver_pot(mut cx: read_driver_pot::Context) {
        let mut debouncer = Debouncer::new();
        let mut adc_val = cx.local.adc.read(cx.local.driver_pot).unwrap();
        adc_val = debouncer.update(adc_val, monotonics::MonoTimer::now().ticks(), 800);

        let new_mode = DriverModes::from((adc_val / 900) as u8);

        defmt::debug!("adc: {} {}", adc_val, new_mode as u8);

        cx.shared.lcd_data.lock(|lcd_data| {
            if new_mode != lcd_data.mode {
                button_driver_mode_handler::spawn(new_mode).unwrap();
            }
        });

        read_driver_pot::spawn_after(Duration::millis(100)).unwrap();
    }

    /// Handle updating of speed
    #[task(shared = [lcd_data], local=[lcd_disp])]
    fn update_display(mut cx: update_display::Context) {
        let lcd_disp = cx.local.lcd_disp;

        cx.shared.lcd_data.lock(|lcd_data| {
            // Display look should as following
            // |L_100_100_100_R|
            // |C E ABCDEFGH  M|
            // Top row: <Left indicator> <battery> <speed> <temperature> <Right indicator>
            // Bot row: <Cruise enabled> <Contactors Engaged> <Warnings> <Driver mode>
            // Start by clearing everything
            let speed = lcd_data.speed;
            let battery = lcd_data.battery;
            let temp = lcd_data.temperature;
            let l_ind = lcd_data.left_indicator;
            let r_ind = lcd_data.right_indicator;
            let warnings = lcd_data.warnings;
            let cruise = lcd_data.cruise;
            let mode = &lcd_data.mode;
            let contactors = lcd_data.contactors;

            lcd_disp.clear_display();
            // TOP ROW
            // Set indicator status
            if l_ind {
                lcd_disp.set_position(0, 0);
                lcd_disp.send_string("L");
            }

            if r_ind {
                lcd_disp.set_position(15, 0);
                lcd_disp.send_string("R");
            }

            let mut data_buf = [0; 32];

            let vehicle_data = (battery as i32).numtoa_str(10, &mut data_buf);

            lcd_disp.set_position(2, 0);
            lcd_disp.send_string(vehicle_data);

            let vehicle_data = (speed as i32).numtoa_str(10, &mut data_buf);

            lcd_disp.set_position(6, 0);
            lcd_disp.send_string(vehicle_data);

            let vehicle_data = (temp as i32).numtoa_str(10, &mut data_buf);

            lcd_disp.set_position(10, 0);
            lcd_disp.send_string(vehicle_data);

            // BOTTOM ROW
            if cruise {
                lcd_disp.set_position(0, 1);
                lcd_disp.send_string("C");
            }

            if contactors {
                lcd_disp.set_position(2, 1);
                lcd_disp.send_string("E");
            }

            // Warnings
            for i in 0..warnings.len() {
                lcd_disp.set_position((4 + i).try_into().unwrap(), 1);
                if warnings[i] == 1 {
                    lcd_disp.send_data(
                        WARNING_CODES
                            .chars()
                            .nth(i)
                            .unwrap()
                            .try_into()
                            .unwrap(),
                    );
                } else {
                    lcd_disp.send_string("_");
                }
            }

            // Driver mode
            lcd_disp.set_position(15, 1);
            match mode {
                DriverModes::Drive => lcd_disp.send_string("D"),
                DriverModes::Neutral => lcd_disp.send_string("N"),
                DriverModes::Reverse => lcd_disp.send_string("R"),
            }

            update_display::spawn_after(Duration::millis(1000)).unwrap();
        });
    }

    /// RX 0 interrupt pending handler.
    #[task(priority = 2, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(mut cx: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");

        cx.shared.can.lock(|can| match can.receive() {
            Ok(frame) => can_receive::spawn(frame).unwrap(),
            _ => {}
        })
    }

    /// RX 1 interrupt pending handler.
    #[task(priority = 2, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(mut cx: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");

        cx.shared.can.lock(|can| match can.receive() {
            Ok(frame) => can_receive::spawn(frame).unwrap(),
            _ => {}
        })
    }

    #[task(priority = 1, shared = [lcd_data], local = [ws22], capacity=100)]
    fn can_receive(mut cx: can_receive::Context, frame: Frame) {
        // defmt::trace!("task: can receive");
        let id = match frame.id() {
            Id::Standard(id) => {
                let ws22 = cx.local.ws22;
                if id.as_raw() >= phln::wavesculptor::ID_BASE {
                    let _res = ws22.receive(frame);
                    cx.shared.lcd_data.lock(|lcd_data| {
                        if let Some(voltage) = ws22.status().bus_voltage {
                            lcd_data.battery = voltage;
                        }

                        if let Some(velocity) = ws22.status().vehicle_velocity {
                            lcd_data.speed = velocity * 3.6;
                        }

                        if let Some(temp) = ws22.status().motor_temperature {
                            lcd_data.temperature = temp;
                        }

                        if let Some(error_flags) = ws22.status().error_flags {
                            process_warnings(error_flags, lcd_data);
                        }
                    });
                }
                return;
            }
            Id::Extended(id) => id,
        };

        let id: j1939::ExtendedId = id.into();
        // defmt::debug!("EXT FRAME: {:?} {:?}", id.to_bits(), frame);
    }

    fn process_warnings(error_flags: ErrorFlags, lcd_data: &mut LcdData) {
        for i in 0..lcd_data.warnings.len() {
            lcd_data.warnings[i] = (error_flags.contains(ErrorFlags::from_bits(1 << i).unwrap())) as u8;
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
