#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;
use example_common::*;

use cortex_m_rt::entry;
use defmt::panic;
use embassy::executor::{task, Executor};
use embassy::time::{Duration, Timer};
use embassy::uart::Uart;
use embassy::util::Forever;
use embedded_hal::digital::v2::*;
use futures::future::{select, Either};
use futures::pin_mut;
use nrf52840_hal::clocks;
use nrf52840_hal::gpio;

use embassy_nrf::{interrupt, pac, rtc, spim};

#[task]
async fn run() {
    info!("running!");

    let p = unsafe { embassy_nrf::pac::Peripherals::steal() };
    let p0 = gpio::p0::Parts::new(p.P0);
    let p1 = gpio::p1::Parts::new(p.P1);

    let pins = spim::Pins {
        sck: p1.p1_09.into_push_pull_output(gpio::Level::Low).degrade(),
        miso: Some(p0.p0_12.into_floating_input().degrade()),
        mosi: Some(p1.p1_04.into_push_pull_output(gpio::Level::Low).degrade()),
    };
    let config = spim::Config {
        pins,
        frequency: spim::Frequency::M1,
        mode: spim::MODE_0,
        orc: 0x00,
    };

    let mut ncs = p0.p0_14.into_push_pull_output(gpio::Level::High);
    let mut nrst = p0.p0_23.into_push_pull_output(gpio::Level::High);
    let spim = spim::Spim::new(p.SPIM3, interrupt::take!(SPIM3), config);
    pin_mut!(spim);

    nrst.set_low().unwrap();
    cortex_m::asm::delay(100000);
    nrst.set_high().unwrap();

    // This is supposed to talk with an ENC28J60 chip
    // but it doesn't work yet.

    /*
    // softreset
    cortex_m::asm::delay(10);
    ncs.set_low().unwrap();
    cortex_m::asm::delay(5);
    spim.as_mut().send_receive(&[0xFF], &mut []).await;
    cortex_m::asm::delay(10);
    ncs.set_high().unwrap();
     */

    cortex_m::asm::delay(100000);

    loop {
        // read ESTAT
        cortex_m::asm::delay(5000);
        ncs.set_low().unwrap();
        cortex_m::asm::delay(5000);
        let mut rx = [0; 5];
        spim.as_mut().send_receive(&[0b111_00011], &mut rx).await;
        cortex_m::asm::delay(5000);
        ncs.set_high().unwrap();
        info!("estat: {:[?]}", rx);
    }

    loop {
        // Switch to bank 3

        cortex_m::asm::delay(10);
        ncs.set_low().unwrap();
        cortex_m::asm::delay(5);
        spim.as_mut().send_receive(&[0b100_11111, 0b11], &mut []).await;
        cortex_m::asm::delay(10);
        ncs.set_high().unwrap();

        // read EREVID
        cortex_m::asm::delay(10);
        ncs.set_low().unwrap();
        cortex_m::asm::delay(5);
        let mut rx = [0; 2];
        spim.as_mut().send_receive(&[0b000_10010], &mut rx).await;
        cortex_m::asm::delay(10);
        ncs.set_high().unwrap();

        info!("erevid: {:u8}", rx[1]);
    }
}

static RTC: Forever<rtc::RTC<pac::RTC1>> = Forever::new();
static ALARM: Forever<rtc::Alarm<pac::RTC1>> = Forever::new();
static EXECUTOR: Forever<Executor> = Forever::new();

#[entry]
fn main() -> ! {
    info!("Hello World!");

    let p = unwrap!(embassy_nrf::pac::Peripherals::take());

    clocks::Clocks::new(p.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    let rtc = RTC.put(rtc::RTC::new(p.RTC1, interrupt::take!(RTC1)));
    rtc.start();

    unsafe { embassy::time::set_clock(rtc) };

    let alarm = ALARM.put(rtc.alarm0());
    let executor = EXECUTOR.put(Executor::new_with_alarm(alarm, cortex_m::asm::sev));

    unwrap!(executor.spawn(run()));

    loop {
        executor.run();
        cortex_m::asm::wfe();
    }
}
