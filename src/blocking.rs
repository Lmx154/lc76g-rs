//! Blocking front-end: the async driver run to completion over an adapter whose futures never pend.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_io::{ErrorType, Read, Write};

use crate::command::Command;
use crate::{Config, Error, Fix, Message, Sentence, Stats, Version};

/// Blocking UART presented as an async one.
struct Uart<U>(U);

impl<U: ErrorType> ErrorType for Uart<U> {
    type Error = U::Error;
}

impl<U: Read> embedded_io_async::Read for Uart<U> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

impl<U: Write> embedded_io_async::Write for Uart<U> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush()
    }
}

fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
}

/// LC76G on a blocking UART. Same methods as [`crate::Lc76g`], without `async`. A read blocks until
/// the UART delivers at least one byte, so a silent line blocks forever unless the UART has a read
/// timeout; with one, the error it returns comes out as [`Error::Uart`] and nothing read is lost.
pub struct Lc76g<U> {
    inner: crate::Lc76g<Uart<U>>,
}

impl<U: ErrorType> Lc76g<U> {
    /// Wraps the UART. Nothing is sent or read until a method is called.
    pub const fn new(uart: U) -> Self {
        Self {
            inner: crate::Lc76g::new(Uart(uart)),
        }
    }

    /// Returns the UART. Bytes read from it but not yet consumed are lost.
    pub fn release(self) -> U {
        self.inner.release().0
    }

    /// See [`crate::Lc76g::stats`].
    pub fn stats(&self) -> Stats {
        self.inner.stats()
    }
}

impl<U: Read> Lc76g<U> {
    /// See [`crate::Lc76g::read_sentence`].
    pub fn read_sentence(&mut self) -> Result<Sentence<'_>, Error<U::Error>> {
        block_on(self.inner.read_sentence())
    }

    /// See [`crate::Lc76g::read`].
    pub fn read(&mut self) -> Result<Message, Error<U::Error>> {
        block_on(self.inner.read())
    }

    /// See [`crate::Lc76g::read_fix`].
    pub fn read_fix(&mut self) -> Result<Fix, Error<U::Error>> {
        block_on(self.inner.read_fix())
    }
}

impl<U: Read + Write> Lc76g<U> {
    /// See [`crate::Lc76g::command`].
    pub fn command(&mut self, command: &Command<'_>) -> Result<(), Error<U::Error>> {
        block_on(self.inner.command(command))
    }

    /// See [`crate::Lc76g::version`].
    pub fn version(&mut self) -> Result<Version, Error<U::Error>> {
        block_on(self.inner.version())
    }

    /// See [`crate::Lc76g::init`].
    pub fn init(&mut self, config: &Config) -> Result<Version, Error<U::Error>> {
        block_on(self.inner.init(config))
    }

    /// See [`crate::Lc76g::save_settings`].
    pub fn save_settings(&mut self) -> Result<(), Error<U::Error>> {
        block_on(self.inner.save_settings())
    }
}
