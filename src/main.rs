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
use embedded_hal::{
    spi::{Mode, Phase, Polarity}
};

use stm32l4xx_hal::{
    can::Can,
    device::{CAN1, SPI1},
    flash::FlashExt,
    gpio::{Alternate, Edge, ExtiPin, Input, OpenDrain, Output, PinExt, PullUp, PushPull, PA4, PA5, PA6, PA7, PA8, PA10, PA11, PA12, PA13, PA14, PB5, PB11, PB13,},
    prelude::*,
    stm32,
    watchdog::IndependentWatchdog,
    spi::Spi,
};

use cortex_m::peripheral::NVIC;

use core::marker::PhantomData;

const FILE_TO_CREATE: &'static str = "CREATE.TXT";

use embedded_sdmmc::{
    BlockSpi, Controller, Directory, SdMmcSpi, TimeSource, Timestamp, Volume
};


const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

struct TimeSink {
    _marker: PhantomData<*const ()>,
}

impl TimeSink {
    fn new() -> Self {
        TimeSink { _marker: PhantomData}
    }
}

impl TimeSource for TimeSink {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp{
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
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
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        led_status: PB13<Output<PushPull>>,
        btn_indicator_left: PA8<Input<PullUp>>,
        btn_indicator_right: PB5<Input<PullUp>>, // TODO figure out which pins
        btn_horn: PA14<Input<PullUp>>,
        // sdmmc_controller : Controller<BlockSpi<Spi<SPI1, (PA5<Alternate<PushPull, 5>>, PA6<Alternate<PushPull, 5>>, PA7<Alternate<PushPull, 5>>)>, PA4<Output<OpenDrain>>>, TimeSink>,
        volume: Volume,
        root_dir: Directory
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
        let led_status = gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // TODO: 8 buttons in total
        // figure out a way to have all these in an array or map {button: pin}
        let btn_indicator_left = {
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

        let btn_indicator_right = {
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

        let btn_horn = {
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

        let mut dc = gpiob
            .pb1
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        let sck = gpioa
            .pa5
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

        let miso = gpioa
            .pa6
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

        let mosi = gpioa
            .pa7
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

        dc.set_low();

        let spi = Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            MODE,
            100.kHz(),
            clocks,
            &mut rcc.apb2
        );

        let spi_cs_pin = gpioa
            .pa4
            .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

        let mut spi_dev = SdMmcSpi::new(spi, spi_cs_pin);
            
        let time_sink: TimeSink = TimeSink::new();

        let mut sdmmc_controller = Controller::new(spi_dev.acquire().unwrap(), time_sink);

        let mut volume = match sdmmc_controller.get_volume(embedded_sdmmc::VolumeIdx(0)) {
            Ok(volume) => volume,
            Err(e) => {
                defmt::debug!("Error getting volume 0: {:?}!", e);
                panic!("Error getting volume 0: {:?}!", e);
            },
        };

        let root_dir = match sdmmc_controller.open_root_dir(&volume) {
            Ok(root_dir) => root_dir,
            Err(e) => {
                defmt::debug!("Error getting root directory on volume 0: {:?}!", e);
                panic!("Error getting root directory on volume 0: {:?}!", e);
            },
        };

        match sdmmc_controller.device().card_size_bytes() {
            Ok(size) => defmt::debug!( "Card size: {}", size),
            Err(e) => defmt::debug!("Error reading card size: {:?}", e),
        }

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

        // sd_card_write::spawn().unwrap();
        run::spawn().unwrap();
        heartbeat::spawn().unwrap();

        (
            Shared {
                can                
            },
            Local {
                watchdog,
                led_status,
                btn_indicator_left,
                btn_indicator_right,
                btn_horn,
                // sdmmc_controller,
                volume,
                root_dir
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

    // #[task(priority = 1, local = [volume, root_dir, sdmmc_controller])]
    // fn sd_card_write(mut cx: sd_card_write::Context) {
    //     let volume = cx.local.volume;
    //     let root_dir = cx.local.root_dir;
    //     let sdmmc_controller = cx.local.sdmmc_controller;

    //     let file = sdmmc_controller.open_file_in_dir(volume, &root_dir, FILE_TO_CREATE, embedded_sdmmc::Mode::ReadWriteCreateOrTruncate);
    //     let mut file = match file {
    //         Ok(file) => file,
    //         Err(e) => {
    //             defmt::debug!("Error creating file: {:?}!", e);
    //             panic!("Error creating 'example.txt': {:?}!", e);
    //         },
    //     };

    //     let bytes_written = match sdmmc_controller.write(volume, &mut file, b"testing file writes.") {
    //         Ok(bytes_written) => bytes_written,
    //         Err(e) => {
    //             defmt::debug!("Error writing to 'example.txt': {:?}!", e);
    //             panic!("Error writing to 'example.txt': {:?}!", e);
    //         },
    //     };
    //     defmt::debug!("Bytes written: {}", bytes_written);
    
    //     sdmmc_controller.close_file(&volume, file);
    //     sdmmc_controller.close_dir(&volume, *root_dir);
    
    //     defmt::debug!("File 'example.txt' has been written to the SD card!");

        
    // }

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
