use core::future::Future;
use core::task::Poll;

use atomic_polyfill::{AtomicU8, Ordering};
use embassy::interrupt::{Interrupt, InterruptExt};
use embassy::util::{AtomicWaker, OnDrop};
use futures::future::poll_fn;

use super::{ReadDma, WriteDma};
use crate::interrupt;
use crate::pac;
use crate::pac::bdma::vals;

const CH_COUNT: usize = pac::peripheral_count!(DMA) * 8;
const CH_STATUS_NONE: u8 = 0;
const CH_STATUS_COMPLETED: u8 = 1;
const CH_STATUS_ERROR: u8 = 2;

struct State {
    ch_wakers: [AtomicWaker; CH_COUNT],
    ch_status: [AtomicU8; CH_COUNT],
}

impl State {
    const fn new() -> Self {
        const AW: AtomicWaker = AtomicWaker::new();
        const AU: AtomicU8 = AtomicU8::new(CH_STATUS_NONE);
        Self {
            ch_wakers: [AW; CH_COUNT],
            ch_status: [AU; CH_COUNT],
        }
    }
}

static STATE: State = State::new();

#[allow(unused)]
pub(crate) async unsafe fn transfer_p2m(
    regs: pac::bdma::Ch,
    state_number: usize,
    src: *const u8,
    dst: &mut [u8],
) {
    // ndtr is max 16 bits.
    assert!(dst.len() <= 0xFFFF);

    // Reset status
    // Generate a DMB here to flush the store buffer (M7) before enabling the DMA
    STATE.ch_status[state_number].store(CH_STATUS_NONE, Ordering::Release);

    let on_drop = OnDrop::new(|| unsafe {
        regs.cr().modify(|w| {
            w.set_tcie(false);
            w.set_teie(false);
            w.set_en(false);
        });
        while regs.cr().read().en() {}
    });

    regs.par().write_value(src as u32);
    regs.mar().write_value(dst.as_mut_ptr() as u32);
    regs.ndtr().write(|w| w.set_ndt(dst.len() as u16));
    regs.cr().write(|w| {
        w.set_psize(vals::Size::BITS8);
        w.set_msize(vals::Size::BITS8);
        w.set_minc(vals::Inc::ENABLED);
        w.set_teie(true);
        w.set_tcie(true);
        w.set_en(true);
    });

    let res = poll_fn(|cx| {
        STATE.ch_wakers[state_number].register(cx.waker());
        match STATE.ch_status[state_number].load(Ordering::Acquire) {
            CH_STATUS_NONE => Poll::Pending,
            x => Poll::Ready(x),
        }
    })
    .await;

    on_drop.defuse();
    // TODO handle error
    assert!(res == CH_STATUS_COMPLETED);
}

#[allow(unused)]
pub(crate) async unsafe fn transfer_m2p(
    regs: pac::bdma::Ch,
    state_number: usize,
    src: &[u8],
    dst: *mut u8,
) {
    // ndtr is max 16 bits.
    assert!(src.len() <= 0xFFFF);

    // Reset status
    // Generate a DMB here to flush the store buffer (M7) before enabling the DMA
    STATE.ch_status[state_number].store(CH_STATUS_NONE, Ordering::Release);

    let on_drop = OnDrop::new(|| unsafe {
        regs.cr().modify(|w| {
            w.set_tcie(false);
            w.set_teie(false);
            w.set_en(false);
        });
        while regs.cr().read().en() {}
    });

    regs.par().write_value(dst as u32);
    regs.mar().write_value(src.as_ptr() as u32);
    regs.ndtr().write(|w| w.set_ndt(src.len() as u16));
    regs.cr().write(|w| {
        w.set_psize(vals::Size::BITS8);
        w.set_msize(vals::Size::BITS8);
        w.set_minc(vals::Inc::ENABLED);
        w.set_dir(vals::Dir::FROMMEMORY);
        w.set_teie(true);
        w.set_tcie(true);
        w.set_en(true);
    });

    let res = poll_fn(|cx| {
        STATE.ch_wakers[state_number].register(cx.waker());
        match STATE.ch_status[state_number].load(Ordering::Acquire) {
            CH_STATUS_NONE => Poll::Pending,
            x => Poll::Ready(x),
        }
    })
    .await;

    on_drop.defuse();
    // TODO handle error
    assert!(res == CH_STATUS_COMPLETED);
}

unsafe fn on_irq() {
    pac::peripherals! {
        (bdma, $dma:ident) => {
                let isr = pac::$dma.isr().read();
                pac::$dma.ifcr().write_value(isr);
                let dman = <crate::peripherals::$dma as sealed::Dma>::num() as usize;

                for chn in 0..7 {
                    let n = dman * 8 + chn;
                    if isr.teif(chn) {
                        STATE.ch_status[n].store(CH_STATUS_ERROR, Ordering::Relaxed);
                        STATE.ch_wakers[n].wake();
                    } else if isr.tcif(chn) {
                        STATE.ch_status[n].store(CH_STATUS_COMPLETED, Ordering::Relaxed);
                        STATE.ch_wakers[n].wake();
                    }
                }

        };
    }
}

/// safety: must be called only once
pub(crate) unsafe fn init() {
    pac::interrupts! {
        (DMA, $irq:ident) => {
            crate::interrupt::$irq::steal().enable();
        };
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Dma {
        fn num() -> u8;
    }

    pub trait Channel {
        fn dma_regs() -> pac::bdma::Dma;

        fn state_num(&self) -> usize;

        fn ch_num(&self) -> u8;

        fn regs(&self) -> pac::bdma::Ch {
            Self::dma_regs().ch(self.ch_num() as usize)
        }
    }
}

pub trait Dma: sealed::Dma + Sized {}
pub trait Channel: sealed::Channel + Sized {}

macro_rules! impl_dma {
    ($peri:ident, $num:expr) => {
        impl Dma for crate::peripherals::$peri {}
        impl sealed::Dma for crate::peripherals::$peri {
            fn num() -> u8 {
                $num
            }
        }
    };
}

macro_rules! impl_dma_channel {
    ($channel_peri:ident, $dma_peri:ident, $dma_num:expr, $ch_num:expr) => {
        impl Channel for crate::peripherals::$channel_peri {}
        impl sealed::Channel for crate::peripherals::$channel_peri {
            #[inline]
            fn dma_regs() -> pac::bdma::Dma {
                crate::pac::$dma_peri
            }

            fn state_num(&self) -> usize {
                ($dma_num * 8) + $ch_num
            }

            fn ch_num(&self) -> u8 {
                $ch_num
            }
        }

        impl<T> WriteDma<T> for crate::peripherals::$channel_peri
        where
            T: 'static,
        {
            type WriteDmaFuture<'a> = impl Future<Output = ()>;

            fn transfer<'a>(&'a mut self, buf: &'a [u8], dst: *mut u8) -> Self::WriteDmaFuture<'a>
            where
                T: 'a,
            {
                use sealed::Channel as _Channel;

                let state_num = self.state_num();
                let regs = self.regs();
                unsafe { transfer_m2p(regs, state_num, buf, dst) }
            }
        }

        impl<T> ReadDma<T> for crate::peripherals::$channel_peri
        where
            T: 'static,
        {
            type ReadDmaFuture<'a> = impl Future<Output = ()>;

            fn transfer<'a>(
                &'a mut self,
                src: *const u8,
                buf: &'a mut [u8],
            ) -> Self::ReadDmaFuture<'a>
            where
                T: 'a,
            {
                use sealed::Channel as _Channel;

                let state_num = self.state_num();
                let regs = self.regs();
                unsafe { transfer_p2m(regs, state_num, src, buf) }
            }
        }
    };
}

pac::peripherals! {
    (bdma, DMA1) => {
        impl_dma!(DMA1, 0);
        pac::dma_channels! {
            ($channel_peri:ident, DMA1, $channel_num:expr) => {
                impl_dma_channel!($channel_peri, DMA1, 0, $channel_num);
            };
        }
    };
    (bdma, DMA2) => {
        impl_dma!(DMA2, 1);
        pac::dma_channels! {
            ($channel_peri:ident, DMA2, $channel_num:expr) => {
                impl_dma_channel!($channel_peri, DMA2, 1, $channel_num);
            };
        }
    };
}

pac::interrupts! {
    (DMA, $irq:ident) => {
        #[crate::interrupt]
        unsafe fn $irq () {
            on_irq()
        }
    };
}
