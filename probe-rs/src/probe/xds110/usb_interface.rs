use std::time::Duration;

use nusb::Interface;

use crate::probe::{
    DebugProbeError, DebugProbeSelector, ProbeCreationError, usb_util::InterfaceExt,
};

use super::{USB_SERIAL_NATIVE_SUFFIX, XDS110_USB_DEVICES, Xds110Error, commands::Xds110Command};

pub struct Xds110UsbDevice {
    device_handle: Interface,
    epout: u8,
    epin: u8,
}

impl Xds110UsbDevice {
    pub fn new_from_selector(selector: &DebugProbeSelector) -> Result<Self, ProbeCreationError> {
        let mut selector = selector.clone();
        // Strip the suffix from the passed probe to make sure it matches the list of devices.
        // If the serial number contains only the suffix, remove the serial number entirely.
        // If there is no serial number, then the probe is not one we recognize.
        let Some(serial) = selector.serial_number else {
            return Err(ProbeCreationError::NotFound);
        };
        let Some(serial) = serial.strip_suffix(USB_SERIAL_NATIVE_SUFFIX) else {
            return Err(ProbeCreationError::NotFound);
        };
        selector.serial_number = Some(serial.to_owned());

        let mut xds110_description = None;
        'usb_list: for device in nusb::list_devices().map_err(ProbeCreationError::Usb)? {
            for xds110 in XDS110_USB_DEVICES {
                if xds110.vid == device.vendor_id()
                    && xds110.pid == device.product_id()
                    && selector.matches(&device)
                {
                    xds110_description = Some((xds110, device));
                    break 'usb_list;
                }
            }
        }
        let Some((xds110, device)) = xds110_description.take() else {
            return Err(ProbeCreationError::NotFound);
        };

        let mut endpoint_out = false;
        let mut endpoint_in = false;

        let device_handle = device.open().map_err(ProbeCreationError::Usb)?;

        let mut configs = device_handle.configurations();
        let Some(config) = configs.next() else {
            tracing::error!("Unable to find default configuration");
            return Err(Xds110Error::ConfigurationNotFound.into());
        };
        let Some(interface) = config
            .interfaces()
            .find(|x| x.interface_number() == xds110.interface)
        else {
            tracing::error!("Unable to find interface {} on XDS110", xds110.interface);
            return Err(Xds110Error::InterfaceNotFound.into());
        };

        for alt_setting in interface.alt_settings() {
            for endpoint in alt_setting.endpoints() {
                if endpoint.address() == xds110.epout {
                    endpoint_out = true;
                } else if endpoint.address() == xds110.epin {
                    endpoint_in = true;
                }
            }
        }

        if !endpoint_out || !endpoint_in {
            tracing::trace!(
                "Couldn't find endpoints (out: {}  in: {})",
                endpoint_out,
                endpoint_in
            );
            return Err(Xds110Error::EndpointNotFound.into());
        }

        tracing::trace!("Aquired handle for probe");
        let device_handle = device_handle
            .claim_interface(xds110.interface)
            .map_err(ProbeCreationError::Usb)?;
        tracing::trace!("Claimed interface 0 of USB device.");

        let usb_wlink = Self {
            device_handle,
            epin: xds110.epin,
            epout: xds110.epout,
        };

        tracing::debug!("Succesfully attached to XDS110.");

        Ok(usb_wlink)
    }

    pub(crate) fn send_command<C: Xds110Command + std::fmt::Debug>(
        &mut self,
        cmd: C,
    ) -> Result<C::Response, DebugProbeError> {
        tracing::trace!("Sending command: {:x?}", cmd);

        let mut rxbuf = [0u8; 4096];
        let len = cmd.to_bytes(&mut rxbuf)?;

        let timeout = Duration::from_millis(10000);

        let written_bytes = self
            .device_handle
            .write_bulk(self.epout, &rxbuf[..len], timeout)
            .map_err(DebugProbeError::Usb)?;

        if written_bytes != len {
            tracing::error!("Not enough bytes written");
            return Err(Xds110Error::NotEnoughBytesWritten {
                is: written_bytes,
                should: len,
            }
            .into());
        }

        let mut rxbuf = [0u8; 4096];
        let read_bytes = self
            .device_handle
            .read_bulk(self.epin, &mut rxbuf[..], timeout)
            .map_err(DebugProbeError::Usb)?;

        if read_bytes < 3 {
            return Err(Xds110Error::NotEnoughBytesRead {
                is: read_bytes,
                should: 3,
            }
            .into());
        }

        if rxbuf[0] != b'*' {
            return Err(Xds110Error::MagicValueNotFound.into());
        }

        let payload_bytes = u16::from_le_bytes(rxbuf[1..=2].try_into().unwrap()) as usize;
        if read_bytes != payload_bytes + 3 {
            return Err(Xds110Error::NotEnoughBytesRead {
                is: read_bytes,
                should: payload_bytes + 3,
            }
            .into());
        }

        let response = cmd.parse_response(&rxbuf[3..read_bytes])?;

        Ok(response)
    }
}
