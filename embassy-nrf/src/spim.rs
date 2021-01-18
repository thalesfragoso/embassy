use core::future::{poll_fn, Future};
use core::ops::Deref;
use core::pin::Pin;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;
use embassy::util::WakerRegistration;

use crate::fmt::{panic, todo, *};
use crate::hal::gpio::Port as GpioPort;
use crate::interrupt::{self, OwnedInterrupt};
use crate::pac;
use crate::util::peripheral::{PeripheralMutex, PeripheralState};

pub use crate::hal::spim::{Frequency, Mode, Phase, Pins, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

#[derive(Debug)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    /// EasyDMA can only read from data memory, read only buffers in flash will fail.
    DMABufferNotInDataMemory,
}

struct State<T: Instance> {
    spim: T,
    waker: WakerRegistration,
}

pub struct Spim<T: Instance> {
    inner: PeripheralMutex<State<T>>,
}

#[cfg(any(feature = "52833", feature = "52840"))]
fn port_bit(port: GpioPort) -> bool {
    match port {
        GpioPort::Port0 => false,
        GpioPort::Port1 => true,
    }
}

pub struct Config {
    pub pins: Pins,
    pub frequency: Frequency,
    pub mode: Mode,
    pub orc: u8,
}

impl<T: Instance> Spim<T> {
    pub fn new(spim: T, irq: T::Interrupt, config: Config) -> Self {
        // Select pins.
        spim.psel.sck.write(|w| {
            let w = unsafe { w.pin().bits(config.pins.sck.pin()) };
            #[cfg(any(feature = "52843", feature = "52840"))]
            let w = w.port().bit(port_bit(config.pins.sck.port()));
            w.connect().connected()
        });

        match config.pins.mosi {
            Some(mosi) => spim.psel.mosi.write(|w| {
                let w = unsafe { w.pin().bits(mosi.pin()) };
                #[cfg(any(feature = "52843", feature = "52840"))]
                let w = w.port().bit(port_bit(mosi.port()));
                w.connect().connected()
            }),
            None => spim.psel.mosi.write(|w| w.connect().disconnected()),
        }
        match config.pins.miso {
            Some(miso) => spim.psel.miso.write(|w| {
                let w = unsafe { w.pin().bits(miso.pin()) };
                #[cfg(any(feature = "52843", feature = "52840"))]
                let w = w.port().bit(port_bit(miso.port()));
                w.connect().connected()
            }),
            None => spim.psel.miso.write(|w| w.connect().disconnected()),
        }

        // Enable SPIM instance.
        spim.enable.write(|w| w.enable().enabled());

        // Configure mode.
        let mode = config.mode;
        spim.config.write(|w| {
            // Can't match on `mode` due to embedded-hal, see https://github.com/rust-embedded/embedded-hal/pull/126
            if mode == MODE_0 {
                w.order().msb_first();
                w.cpol().active_high();
                w.cpha().leading();
            } else if mode == MODE_1 {
                w.order().msb_first();
                w.cpol().active_high();
                w.cpha().trailing();
            } else if mode == MODE_2 {
                w.order().msb_first();
                w.cpol().active_low();
                w.cpha().leading();
            } else {
                w.order().msb_first();
                w.cpol().active_low();
                w.cpha().trailing();
            }
            w
        });

        // Configure frequency.
        let frequency = config.frequency;
        spim.frequency.write(|w| w.frequency().variant(frequency));

        // Set over-read character
        let orc = config.orc;
        spim.orc.write(|w|
            // The ORC field is 8 bits long, so any u8 is a valid value to write.
            unsafe { w.orc().bits(orc) });

        // Disable all events interrupts
        spim.intenclr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });

        let state = State {
            spim,
            waker: WakerRegistration::new(),
        };
        Self {
            inner: PeripheralMutex::new(state, irq),
        }
    }

    fn inner(self: Pin<&mut Self>) -> Pin<&mut PeripheralMutex<State<T>>> {
        unsafe { Pin::new_unchecked(&mut self.get_unchecked_mut().inner) }
    }

    pub fn free(self: Pin<&mut Self>) -> (T, T::Interrupt) {
        let (mut state, irq) = self.inner().free();
        (state.spim, irq)
    }

    pub fn send_receive<'a>(mut self: Pin<&'a mut Self>, tx: &'a [u8], rx: &'a mut [u8]) -> impl Future<Output = ()> + 'a {
        async move {
            self.as_mut().inner().with(|s, irq| {
                // Conservative compiler fence to prevent optimizations that do not
                // take in to account actions by DMA. The fence has been placed here,
                // before any DMA action has started.
                compiler_fence(Ordering::SeqCst);

                // Set up the DMA write.
                s.spim.txd.ptr.write(|w| unsafe { w.ptr().bits(tx.as_ptr() as u32) });
                s.spim.txd.maxcnt.write(|w| unsafe { w.maxcnt().bits(tx.len() as _) });

                // Set up the DMA read.
                s.spim.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx.as_mut_ptr() as u32) });
                s.spim.rxd.maxcnt.write(|w| unsafe { w.maxcnt().bits(rx.len() as _) });

                // Reset and enable the event
                s.spim.events_end.write(|w| w);
                s.spim.intenset.write(|w| w.end().set());

                // Start SPI transaction.
                s.spim.tasks_start.write(|w| unsafe { w.bits(1) });

                // Conservative compiler fence to prevent optimizations that do not
                // take in to account actions by DMA. The fence has been placed here,
                // after all possible DMA actions have completed.
                compiler_fence(Ordering::SeqCst);
            });

            // Wait for 'end' event.
            poll_fn(|cx| {
                self.as_mut().inner().with(|s, irq| {
                    if s.spim.events_end.read().bits() != 0 {
                        return Poll::Ready(());
                    }
                    s.waker.register(cx.waker());
                    Poll::Pending
                })
            })
            .await;
        }
    }
}

impl<U: Instance> Drop for Spim<U> {
    fn drop(&mut self) {
        let inner = unsafe { Pin::new_unchecked(&mut self.inner) };
        if let Some((mut state, _irq)) = inner.try_free() {
            panic!("todo");
        }
    }
}

impl<U: Instance> PeripheralState for State<U> {
    type Interrupt = U::Interrupt;
    fn on_interrupt(&mut self) {
        if self.spim.events_end.read().bits() != 0 {
            self.spim.intenclr.write(|w| w.end().clear());
            self.waker.wake()
        }
    }
}

mod sealed {
    pub trait Instance {}

    impl Instance for crate::pac::SPIM0 {}
    impl Instance for crate::pac::SPIM1 {}
    impl Instance for crate::pac::SPIM2 {}
    impl Instance for crate::pac::SPIM3 {}
}

pub trait Instance: Deref<Target = pac::spim0::RegisterBlock> + sealed::Instance {
    type Interrupt: OwnedInterrupt;
}

impl Instance for pac::SPIM0 {
    type Interrupt = interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0Interrupt;
}
impl Instance for pac::SPIM1 {
    type Interrupt = interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1Interrupt;
}
impl Instance for pac::SPIM2 {
    type Interrupt = interrupt::SPIM2_SPIS2_SPI2Interrupt;
}
impl Instance for pac::SPIM3 {
    type Interrupt = interrupt::SPIM3Interrupt;
}
