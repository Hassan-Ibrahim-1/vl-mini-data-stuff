use cortex_m::singleton;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri, bind_interrupts,
    can::{
        self, Can, CanRx, CanTx, Frame,
        enums::{BusError, FrameCreateError},
        frame::Envelope,
    },
    peripherals::{CAN1, PA11, PA12},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use firmware_common_new::can_bus::{
    CanBusFrame, CanBusRX, CanBusTX,
    id::{CanBusExtendedId, can_node_id_from_serial_number},
    messages::DATA_TRANSFER_MESSAGE_TYPE,
    node_types::VL_MINI_NODE_TYPE,
    receiver::CanReceiver,
    sender::CanSender,
};
use stm32_device_signature::device_id;

pub async fn start_can_bus_tasks(
    spawner: &Spawner,
    can1: Peri<'static, CAN1>,
    pa11: Peri<'static, PA11>,
    pa12: Peri<'static, PA12>,
) -> (
    &'static CanSender<NoopRawMutex>,
    &'static CanReceiver<NoopRawMutex, 4, 4>,
) {
    let can_node_id = can_node_id_from_serial_number(device_id());
    info!("CAN Device ID: {}", can_node_id);

    let can_sender =
        singleton!(: CanSender<NoopRawMutex> = CanSender::new(VL_MINI_NODE_TYPE, can_node_id,  Some(&defmt_rtt_pipe::PIPE)))
            .unwrap();
    let can_receiver =
        singleton!(: CanReceiver<NoopRawMutex, 4, 4> = CanReceiver::new(can_node_id)).unwrap();

    bind_interrupts!(struct Irqs {
        CAN1_RX0 => can::Rx0InterruptHandler<CAN1>;
        CAN1_RX1 => can::Rx1InterruptHandler<CAN1>;
        CAN1_TX  => can::TxInterruptHandler<CAN1>;
        CAN1_SCE => can::SceInterruptHandler<CAN1>;
    });

    let mut can = Can::new(can1, pa11, pa12, Irqs);
    can.set_bitrate(1_000_000);
    can.enable().await;
    let (tx, rx) = can.split();

    spawner.must_spawn(can_bus_tx_task(can_sender, tx));
    spawner.must_spawn(can_bus_rx_task(can_receiver, rx));

    (can_sender, can_receiver)
}

#[embassy_executor::task]
async fn can_bus_tx_task(can_sender: &'static CanSender<NoopRawMutex>, tx: CanTx<'static>) {
    struct TxWrapper(CanTx<'static>);
    impl CanBusTX for TxWrapper {
        type Error = FrameCreateError;

        async fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
            let frame = Frame::new_extended(id, data)?;
            self.0.write(&frame).await;
            Ok(())
        }
    }

    let mut tx_wrapper = TxWrapper(tx);
    can_sender.run_daemon(&mut tx_wrapper).await;
}

#[embassy_executor::task]
async fn can_bus_rx_task(
    can_receiver: &'static CanReceiver<NoopRawMutex, 4, 4>,
    rx: CanRx<'static>,
) {
    struct RxWrapper(CanRx<'static>);
    struct EnvelopeWrapper(Envelope);

    impl CanBusFrame for EnvelopeWrapper {
        fn timestamp_us(&self) -> u64 {
            self.0.ts.as_micros()
        }

        fn id(&self) -> u32 {
            match self.0.frame.id() {
                embedded_can::Id::Standard(standard_id) => standard_id.as_raw() as u32,
                embedded_can::Id::Extended(extended_id) => extended_id.as_raw() as u32,
            }
        }

        fn data(&self) -> &[u8] {
            self.0.frame.data()
        }
    }

    impl CanBusRX for RxWrapper {
        type Error = BusError;
        type Frame = EnvelopeWrapper;

        async fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
            loop {
                let frame = self.0.read().await.map(EnvelopeWrapper)?;
                let id = CanBusExtendedId::from_raw(frame.id());

                // filter out data transfer messages so we don't waste time decoding them
                if id.message_type != DATA_TRANSFER_MESSAGE_TYPE {
                    return Ok(frame);
                }
            }
        }
    }

    let mut rx_wrapper = RxWrapper(rx);
    can_receiver.run_daemon::<_, 8>(&mut rx_wrapper).await;
}
