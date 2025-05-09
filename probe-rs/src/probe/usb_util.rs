use async_io::{Timer, block_on};
use futures_lite::FutureExt;
use nusb::{Interface, transfer::RequestBuffer};
use std::{io, time::Duration};

pub trait InterfaceExt {
    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;
    fn write_bulk(&self, endpoint: u8, buf: &[u8], timeout: Duration) -> io::Result<usize>;
}

impl InterfaceExt for Interface {
    fn write_bulk(&self, endpoint: u8, buf: &[u8], timeout: Duration) -> io::Result<usize> {
        let fut = async {
            tracing::trace!("Writing data: {:x?}", buf);
            let comp = self.bulk_out(endpoint, buf.to_vec()).await;
            comp.status.map_err(io::Error::other)?;

            let n = comp.data.actual_length();
            Ok(n)
        };

        block_on(fut.or(async {
            Timer::after(timeout).await;
            Err(std::io::ErrorKind::TimedOut.into())
        }))
    }

    fn read_bulk(&self, endpoint: u8, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        let fut = async {
            if buf.len() > 64 {
                let mut queue = self.bulk_in_queue(endpoint);
                let n_transfers = buf.len() / 64;
                let transfer_size = buf.len();
                while queue.pending() < n_transfers {
                    queue.submit(RequestBuffer::new(transfer_size));
                }

                let mut offset = 0;
                let mut finished = false;
                while offset < transfer_size && !finished {
                    let comp = queue.next_complete().await;
                    comp.status.map_err(io::Error::other)?;

                    let n = comp.data.len();
                    buf[offset..offset + n].copy_from_slice(&comp.data);
                    offset += n;
                    // Finish on ZLP or non-full buffer
                    finished = n == 0 || n != 64;
                    queue.submit(RequestBuffer::reuse(comp.data, transfer_size));
                }
                tracing::trace!("Read data: {:x?}", &buf[..offset]);
                Ok(offset)
            } else {
                let comp = self.bulk_in(endpoint, RequestBuffer::new(buf.len())).await;
                comp.status.map_err(io::Error::other)?;

                let n = comp.data.len();
                buf[..n].copy_from_slice(&comp.data);
                tracing::trace!("Read data: {:x?}", &buf[..n]);
                Ok(n)
            }
        };

        block_on(fut.or(async {
            Timer::after(timeout).await;
            Err(std::io::ErrorKind::TimedOut.into())
        }))
    }
}
