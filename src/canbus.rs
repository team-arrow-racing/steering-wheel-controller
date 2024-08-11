use crate::app::*;
use fdcan::{frame::RxFrameInfo, id::Id, interrupt::Interrupt};
use rtic::Mutex;
use stm32h7xx_hal::nb::block;

pub fn can_rx0_pending(mut cx: can_rx0_pending::Context) {
    defmt::trace!("RX0 received");
    cx.shared.can.lock(|can| {
        let mut buffer = [0_u8; 8];

        if can.has_interrupt(Interrupt::RxFifo0NewMsg) {
            match block!(can.receive0(&mut buffer)) {
                Ok(rxframe) => {
                    can_receive::spawn(rxframe.unwrap(), buffer).ok()
                },
                Err(_) => {
                    defmt::trace!("Error");
                    Some(())
                }
            };

            can.clear_interrupt(Interrupt::RxFifo0NewMsg);
        }
    });
}

pub fn can_rx1_pending(mut cx: can_rx1_pending::Context) {
    defmt::trace!("RX1 received");
    cx.shared.can.lock(|can| {
        let mut buffer = [0_u8; 8];

        if can.has_interrupt(Interrupt::RxFifo1NewMsg) {
            match block!(can.receive1(&mut buffer)) {
                Ok(rxframe) => {
                    can_receive::spawn(rxframe.unwrap(), buffer).ok()
                },
                Err(_) => {
                    defmt::trace!("Error");
                    Some(())
                }
            };

            can.clear_interrupt(Interrupt::RxFifo1NewMsg);
        }
    });
}

pub async fn can_receive(_cx: can_receive::Context<'_>, frame: RxFrameInfo, buffer: [u8; 8]) {
    let id = frame.id;
    match id {
        Id::Standard(id) => defmt::info!("Received Header: {:#02x}", id.as_raw()),
        Id::Extended(id) => defmt::info!("Received Header: {:#03x}", id.as_raw()),
    };
    defmt::info!("received data: {:#02x}", buffer);
}
