#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod bootloader;
use core::{mem, ptr};
use embassy_futures::select::Either;
use {defmt_rtt_pipe as _, panic_probe as _};

use cortex_m::{asm, singleton};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    crc::{Config as CrcConfig, Crc, InputReverseConfig, PolySize},
    peripherals::{ADC1, DMA1_CH1, PA0, PA10, PA2, PB11, PB12, PB14, PB6, PB7, PC13},
    Peri,
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    time::mhz,
};

use crate::bootloader::{configure_next_boot, watchdog_task, BootOption};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    mutex::Mutex,
    pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult},
    watch::Watch,
};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::{peripherals::PB3, time::Hertz};
use embedded_sdmmc::asynchronous::{Block, BlockDevice as _, BlockIdx, SdCard};
use firmware_common_new::can_bus::{
    custom_status::ozys_custom_status::OzysCustomStatus,
    messages::{
        node_status::{NodeHealth, NodeMode},
        unix_time::UnixTimeMessage,
        vl_status::{FlightStage, VLStatusMessage},
        CanBusMessageEnum,
    },
    receiver::CanReceiver,
};
use firmware_common_new::can_bus::{
    messages::{
        node_status::NodeStatusMessage, ozys_measurement::OzysMeasurementMessage,
        reset::ResetMessage,
    },
    sender::CanSender,
};
mod can;
use can::start_can_bus_tasks;

pub enum BlockReady {
    BlockWaiting = 0,
    BlockReady = 1,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = {
        use embassy_stm32::rcc::mux::*;
        use embassy_stm32::rcc::*;
        let mut config = embassy_stm32::Config::default();
        let rcc = &mut config.rcc;

        rcc.hsi = false;
        rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });
        rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });

        // Limit clock speed to 150Mhz to reduce power consumption
        rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL75,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV2),
            divr: Some(PllRDiv::DIV2),
        });

        rcc.sys = Sysclk::PLL1_R;
        rcc.ahb_pre = AHBPrescaler::DIV1;
        rcc.apb1_pre = APBPrescaler::DIV1;
        rcc.apb2_pre = APBPrescaler::DIV1;

        rcc.ls = LsConfig::default_lse();

        rcc.mux.rtcsel = RtcClockSource::LSE;
        rcc.mux.adc12sel = Adcsel::SYS;
        rcc.mux.adc345sel = Adcsel::SYS;
        rcc.mux.clk48sel = Clk48sel::HSI48;
        rcc.mux.fdcansel = Fdcansel::PLL1_Q;

        config
    };
    let p = embassy_stm32::init(config);
    info!("Hello OZYS V3!");

    // Peripherals

    // TOP TO BOTTOM ORDERING OF LEDS
    // led 2: PA10
    // led 3: PB6
    // led 4: PB7
    // led 1: PC13
    // status led: PB14

    let ps = Output::new(p.PA7, Level::Low, Speed::Low); // PS pin, low: force pwm
    mem::forget(ps);

    let (can_sender, can_receiver) = start_can_bus_tasks(&spawner, p.FDCAN3, p.PA8, p.PA15).await;
    spawner.must_spawn(watchdog_task(p.IWDG));
    spawner.must_spawn(node_status_task(can_sender));
    spawner.must_spawn(status_led_task(p.PB14));

    // sd
    let cs = Output::new(p.PB9, Level::High, Speed::High); // needed to configure as spi mode on peripheral
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(5_000_000);
    let spi1: Spi<'static, embassy_stm32::mode::Async> = Spi::new(
        p.SPI1,
        unsafe { PB3::steal() },
        p.PB5,
        p.PB4,
        p.DMA1_CH2,
        p.DMA1_CH3,
        spi_config,
    );
    let spi1 = singleton!(:Mutex<NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>> = Mutex::new(spi1)).unwrap();
    let crc_config =
        CrcConfig::new(InputReverseConfig::None, false, PolySize::Width32, 69, 69).unwrap();
    let crc = Crc::new(p.CRC, crc_config);

    let sd = SpiDevice::new(spi1, cs);
    let sdcard = singleton!(: SdCard<SpiDevice<'static, NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>, Output<'static>>, Delay> = SdCard::new(sd, Delay)).unwrap();
    spawner.must_spawn(sd_card_read_all_blocks(sdcard, crc));
    spawner.must_spawn(can_reset_task(can_receiver, sdcard));
}

#[embassy_executor::task]
async fn sd_card_read_all_blocks(
    sdcard: &'static SdCard<
        SpiDevice<'static, NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>, Output<'static>>,
        Delay,
    >,
    mut crc: Crc<'static>,
) {
    let size: u64 = sdcard.num_bytes().await.unwrap();
    let block_count = (size / 512) as u32;
    info!("Card size is {} bytes, {} blocks", size, block_count);
    let mut highest_card_index;
    let mut read_block = [Block::default()];
    let block_index_primary = 0;
    let block_index_secondary = 1;
    let mut block_index_primary_inside = 0;
    let mut block_index_secondary_inside = 1;
    let mut crc_primary_flag = false; // check for corruption, false = corrupted
    let mut crc_secondary_flag = false;
    let mut write_to_primary = true;

    //TEST PRIMARY CRC
    sdcard
        .read(&mut read_block, BlockIdx(block_index_primary))
        .await
        .unwrap();
    info!("Read Prim: {}", read_block[0].contents);
    crc.reset();
    let check_sum = crc.feed_bytes(&read_block[0][0..508]);
    let check_sum2 = u32::from_le_bytes(read_block[0][508..512].try_into().unwrap());
    if check_sum != check_sum2 {
        info!(
            "Primary Failed, checksum mismatch: {} != {}",
            check_sum.to_le_bytes(),
            check_sum2.to_le_bytes()
        );
    } else {
        info!("Primary Succeeded");
        block_index_primary_inside = u32::from_le_bytes(read_block[0][0..4].try_into().unwrap());
        crc_primary_flag = true;
    }

    //TEST SECONDARY CRC
    sdcard
        .read(&mut read_block, BlockIdx(block_index_secondary))
        .await
        .unwrap();
    info!("Read Sec: {}", read_block[0].contents);
    crc.reset();
    let check_sum = crc.feed_bytes(&read_block[0][0..508]);
    let check_sum2 = u32::from_le_bytes(read_block[0][508..512].try_into().unwrap());
    if check_sum != check_sum2 {
        info!(
            "Secondary Failed, checksum mismatch: {} != {}",
            check_sum.to_be_bytes(),
            check_sum2.to_le_bytes()
        );
    } else {
        info!("Secondary Succeeded");
        block_index_secondary_inside = u32::from_le_bytes(read_block[0][0..4].try_into().unwrap());
        crc_secondary_flag = true;
    }
    if crc_secondary_flag && crc_primary_flag {
        if block_index_primary_inside > block_index_secondary_inside {
            highest_card_index = block_index_primary_inside;
            write_to_primary = false;
        } else {
            highest_card_index = block_index_secondary_inside;
        }
    } else if crc_primary_flag {
        highest_card_index = block_index_primary_inside;
        write_to_primary = false;
    } else if crc_secondary_flag {
        highest_card_index = block_index_secondary_inside;
    } else {
        info!("Primary and Secondary Block Indices are Corrupted/Missing");
        highest_card_index = block_count;
    }
    let mut current_index = 2;
    info!("highest card index = {}", highest_card_index);
    while current_index <= highest_card_index {
        //info!("card index = {}, out of {}", current_index,highest_card_index);
        //crc.reset();
        let block_id = BlockIdx(current_index);
        sdcard.read(&mut read_block, block_id).await.unwrap();
        /*
        info!(
            "{} + check: {}",
            &read_block[0].contents,
            check_sum.to_le_bytes()
        );
        */
        info!("{}", &read_block[0].contents);
        /*
        let mut block_index = 8;
        while block_index < 504 {
            let mut samples: [f32;4] = [0.0;4];
            for i in (0..4){
                let mut bytes = &read_block[0].contents[block_index+i*4..block_index+4*i+4];
                samples[0] = f32::from_le_bytes(bytes.try_into().unwrap());
            }
            block_index += 16;
            info!("{},{},{},{}",samples[0],samples[1],samples[2],samples[3]);
            Timer::after_millis(10).await;
        }
        */
        current_index += 1;
    }
    asm::bkpt();
}

#[embassy_executor::task]
async fn status_led_task(yellow_led: Peri<'static, PB14>) {
    let mut yellow_led = Output::new(yellow_led, Level::High, Speed::Low);
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        yellow_led.set_high();
        Timer::after_millis(50).await;
        yellow_led.set_low();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn node_status_task(can_sender: &'static CanSender<NoopRawMutex>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        can_sender.send(
            NodeStatusMessage::new(
                Instant::now().as_secs() as u32,
                NodeHealth::Healthy,
                NodeMode::Operational,
                OzysCustomStatus::new(true, true, true, true, true, 0.0),
            )
            .into(),
        );
        ticker.next().await;
    }
}

#[embassy_executor::task] // CHANGE SUBSCRIBER LIMIT??
async fn can_reset_task(
    can_receiver: &'static CanReceiver<NoopRawMutex, 4, 4>,
    sdcard: &'static SdCard<
        SpiDevice<'static, NoopRawMutex, Spi<'static, embassy_stm32::mode::Async>, Output<'static>>,
        Delay,
    >,
) {
    let mut subscriber = can_receiver.subscriber().unwrap();
    loop {
        let can_message = subscriber.next_message_pure().await.data.message;
        if let CanBusMessageEnum::Reset(ResetMessage {
            node_id,
            reset_all,
            into_bootloader,
        }) = can_message
            && (node_id == can_receiver.self_node_id() || reset_all)
        {
            let card_index: i32 = 2;
            let mut empty = [Block::new()];
            empty[0].contents.fill(1);
            empty[0][0..4].copy_from_slice(&card_index.to_le_bytes());
            sdcard.write(&empty, BlockIdx(0)).await.unwrap();
            sdcard.write(&empty, BlockIdx(1)).await.unwrap();
            configure_next_boot(if into_bootloader {
                BootOption::Bootloader
            } else {
                BootOption::Application
            });
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}
