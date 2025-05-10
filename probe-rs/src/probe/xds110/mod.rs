//! XDS110 probe support.
//!
//! The protocol is mostly undocumented, though openocd does have support.

use std::fmt;

use bitvec::{
    bitvec,
    order::Lsb0,
    vec::BitVec,
    view::{AsBits, BitView},
};
use commands::JtagTransit;
use nusb::{DeviceInfo, MaybeFuture};
use probe_rs_target::ScanChainElement;

use self::usb_interface::Xds110UsbDevice;
use super::{
    JtagAccess, ProbeStatistics, RawSwdIo,
    common::{JtagState, RegisterState},
};
use crate::{
    architecture::arm::{ArmCommunicationInterface, communication_interface::DapProbe},
    probe::{
        ChainParams, DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector,
        JtagDriverState, ProbeError, ProbeFactory, SwdSettings, WireProtocol,
        common::{common_sequence, extract_idcodes, extract_ir_lengths},
    },
};

mod commands;
mod usb_interface;

const TCK_FREQ_SLOPE: f64 = 15_100_000.0;
const TCK_FREQ_INTERCEPT: f64 = -1.02;

/// This suffix is added to the serial number to disambiguate it from
/// the CMSIS-DAP version.
pub(crate) const USB_SERIAL_NATIVE_SUFFIX: &str = ".native";

#[derive(Debug)]
pub(crate) struct Xds110UsbDeviceMatch {
    vid: u16,
    pid: u16,
    epin: u8,
    epout: u8,
    interface: u8,
}

pub(crate) const XDS110_USB_DEVICES: &[Xds110UsbDeviceMatch] = &[
    Xds110UsbDeviceMatch {
        vid: 0x0451,
        pid: 0xbef3,
        epin: 0x83,
        epout: 0x02,
        interface: 2,
    },
    Xds110UsbDeviceMatch {
        vid: 0x0451,
        pid: 0xbef4,
        epin: 0x83,
        epout: 0x02,
        interface: 2,
    },
    Xds110UsbDeviceMatch {
        vid: 0x1cbe,
        pid: 0x02a5,
        epin: 0x81,
        epout: 0x01,
        interface: 0,
    },
];

/// Factory for creating [`Xds110`] probes.
#[derive(Debug)]
pub struct Xds110Factory;

impl std::fmt::Display for Xds110Factory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Xds110")
    }
}

impl ProbeFactory for Xds110Factory {
    fn open(&self, selector: &DebugProbeSelector) -> Result<Box<dyn DebugProbe>, DebugProbeError> {
        tracing::trace!("XDS110 Factory is trying to open {:?}", selector);
        let device = Xds110UsbDevice::new_from_selector(selector)?;
        let mut xds110 = Xds110 {
            device,
            name: "XDS110".into(),
            version: [0u8; 4],
            hardware: 0,
            speed: 4_000_000,
            idle_cycles: 0,
            swd_settings: SwdSettings::default(),
            probe_statistics: ProbeStatistics::default(),
            out_accumulator: vec![],
            out_accumulator_bits: 0,
            response: BitVec::new(),
            in_accumulator_capture: BitVec::new(),
            virtual_jtag_state: JtagState::Reset,
            jtag_state: JtagDriverState::default(),
            current_ir: None,
        };

        xds110.init()?;

        Ok(Box::new(xds110))
    }

    fn list_probes(&self) -> Vec<DebugProbeInfo> {
        list_xds110_devices()
    }
}

/// An XDS110 probe
pub struct Xds110 {
    device: Xds110UsbDevice,
    name: String,
    version: [u8; 4],
    hardware: u16,
    speed: u32,
    virtual_jtag_state: JtagState,
    swd_settings: SwdSettings,
    probe_statistics: ProbeStatistics,
    idle_cycles: u8,
    out_accumulator: Vec<u8>,
    out_accumulator_bits: usize,
    response: BitVec,
    in_accumulator_capture: BitVec,
    jtag_state: JtagDriverState,
    current_ir: Option<u32>,
}

impl fmt::Debug for Xds110 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Xds110")
            .field("name", &self.name)
            .field("version", &self.version)
            .field("hardware", &self.hardware)
            .field("speed", &self.speed)
            .field("idle_cycles", &self.idle_cycles)
            .finish()
    }
}

impl Xds110 {
    fn get_probe_info(&mut self) -> Result<(), DebugProbeError> {
        let probe_info = self.device.send_command(commands::GetProbeInfo)?;
        self.version = probe_info.version;
        self.hardware = probe_info.hardware;

        Ok(())
    }

    fn init(&mut self) -> Result<(), DebugProbeError> {
        tracing::debug!("Initializing XDS110...");

        let connect_response = self.device.send_command(commands::Connect)?;
        if connect_response.result != 0 {
            tracing::debug!("Unexpected `connect` response: {}", connect_response.result);
            return Err(DebugProbeError::Other(format!(
                "Unexpected response to `Connect`: {}",
                connect_response.result
            )));
        }

        self.get_probe_info()?;

        tracing::info!(
            "XDS110 firmware version: {:x}.{:x}.{:x}.{:x}, hardware {}",
            self.version[3],
            self.version[2],
            self.version[1],
            self.version[0],
            self.hardware
        );

        self.name = format!(
            "{:x}.{:x}.{:x}.{:x} hardware {}",
            self.version[3], self.version[2], self.version[1], self.version[0], self.hardware
        );

        let trst_value = self.device.send_command(commands::GetTrst)?;
        if trst_value.result != 0 {
            tracing::error!("Couldn't get nTRST value: {:08x}", trst_value.result);
        } else {
            tracing::info!("nTRST set? {}", trst_value.trst);
        }
        self.device.send_command(commands::SetTrst(true))?;

        Ok(())
    }

    fn hz_to_tck_delay(hz: u32) -> u32 {
        let hz = hz.max(1000);

        // Calculate the delay count to set the frequency. These values
        // were verified on an oscilloscope.
        let period = 1.0 / (hz as f64);
        let delay = TCK_FREQ_SLOPE * period + TCK_FREQ_INTERCEPT;

        delay as u32
    }

    fn tck_delay_to_hz(tck: u32) -> u32 {
        if tck == 0 {
            14_000_000
        } else {
            (1.0 / (tck as f64 + TCK_FREQ_INTERCEPT) * TCK_FREQ_SLOPE) as u32
        }
    }

    fn accumulate_bit(
        &mut self,
        tms: bool,
        tdi: bool,
        capture: bool,
    ) -> Result<(), DebugProbeError> {
        let initial_state = self.virtual_jtag_state;

        // If we start in a state where we're interested in capturing bits,
        // add them to the accumulator.
        if matches!(
            initial_state,
            JtagState::Dr(RegisterState::Shift) | JtagState::Ir(RegisterState::Shift)
        ) {
            let accumulator_byte = self.out_accumulator_bits / 8;
            let accumulator_bit = self.out_accumulator_bits & 7;
            if accumulator_bit == 0 {
                self.out_accumulator.push(0);
            }
            if tdi {
                self.out_accumulator[accumulator_byte] |= 1 << accumulator_bit;
            }
            self.out_accumulator_bits += 1;
            self.in_accumulator_capture.push(capture);
        } else if capture {
            tracing::warn!("Not capturing bit because we're not in a state to shift data");
        }

        self.virtual_jtag_state.update(tms);

        // If we just got into a state where we need to transmit data, do so.
        if matches!(
            initial_state,
            JtagState::Dr(RegisterState::Shift) | JtagState::Ir(RegisterState::Shift)
        ) && matches!(
            self.virtual_jtag_state,
            JtagState::Dr(RegisterState::Exit1) | JtagState::Ir(RegisterState::Exit1)
        ) {
            tracing::trace!(
                "Got a full {} register. Updating to: {:x?}",
                if initial_state == JtagState::Dr(RegisterState::Shift) {
                    "DR"
                } else {
                    "IR"
                },
                self.out_accumulator
            );
            let result = self.device.send_command(commands::JtagScan {
                bits: self.out_accumulator_bits as u16,
                path: initial_state,
                trans1: JtagTransit::ViaIdle,
                end_state: JtagState::Idle,
                trans2: JtagTransit::ViaIdle,
                pre: 0,
                post: 0,
                delay: 0,
                rep: 1,
                out_data: self.out_accumulator.clone(),
                response_length: self.out_accumulator.len() as u16,
            })?;
            self.probe_statistics.report_io();

            let want_capture = self
                .in_accumulator_capture
                .iter()
                .find(|v| *v == true)
                .is_some();

            // Decompose the bytes from the result into a bitvec
            if want_capture && !self.response.is_empty() {
                tracing::warn!("response accumulator is not empty");
                self.response.clear();
            }

            let mut debug_display = 0u128;
            let mut captured_bits = 0;

            let tdo_slice = &result.data.view_bits::<Lsb0>()[..self.out_accumulator_bits];

            // tracing::trace!("In accumulator capture: {:?}", self.in_accumulator_capture);
            // tracing::trace!("TDO slice: {:?}", tdo_slice);
            // tracing::trace!(
            //     "Going to iterate through {} iac and {} tdo",
            //     self.in_accumulator_capture.len(),
            //     tdo_slice.len()
            // );

            for (capture, tdo) in self.in_accumulator_capture.drain(..).zip(tdo_slice) {
                if capture {
                    if *tdo && captured_bits < 128 {
                        debug_display |= 1 << captured_bits;
                    }
                    captured_bits += 1;
                    self.response.push(*tdo);
                }
            }
            if captured_bits != 0 {
                tracing::trace!("Read in {} bits. Value: {:x}", captured_bits, debug_display);
            }
            if !self.in_accumulator_capture.is_empty() {
                tracing::error!("In Accumulator isn't empty!");
            }

            tracing::trace!("Result from jtag scan: {:x?}", result);
            self.out_accumulator.clear();
            self.out_accumulator_bits = 0;
        }
        Ok(())
    }
}

impl DebugProbe for Xds110 {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn speed_khz(&self) -> u32 {
        self.speed / 1000
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let tck_delay = Self::hz_to_tck_delay(speed_khz * 1000);
        self.device.send_command(commands::SetTckDelay(tck_delay))?;
        self.speed = Self::tck_delay_to_hz(tck_delay);
        Ok(self.speed_khz())
    }

    /// Attach chip
    fn attach(&mut self) -> Result<(), DebugProbeError> {
        tracing::debug!(
            "Attaching to target system (clock = {}kHz) with scan chain {:?}",
            self.speed / 1000,
            self.jtag_state.scan_chain
        );

        Ok(())
    }

    fn detach(&mut self) -> Result<(), crate::Error> {
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        Ok(())
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        tracing::info!("target reset assert");
        Ok(())
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        tracing::info!("target reset deassert");
        Ok(())
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        self.device.send_command(commands::SetTrst(true))?;
        if protocol == WireProtocol::Jtag {
            // Connect to the JTAG machinery and go to the RESET state
            self.virtual_jtag_state = JtagState::Reset;
            self.device.send_command(commands::CjtagConnect(1))?;
            self.device.send_command(commands::GotoJtagState {
                state: self.virtual_jtag_state,
                transit: JtagTransit::Quickest,
            })?;
            Ok(())
        } else {
            Err(DebugProbeError::NotImplemented {
                function_name: "select_protocol(Swd)",
            })
        }
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        Some(WireProtocol::Jtag)
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn has_arm_interface(&self) -> bool {
        true
    }

    fn try_get_arm_interface<'probe>(
        self: Box<Self>,
    ) -> Result<
        Box<dyn crate::architecture::arm::communication_interface::UninitializedArmProbe + 'probe>,
        (Box<dyn DebugProbe>, DebugProbeError),
    > {
        Ok(Box::new(ArmCommunicationInterface::new(self, false)))
    }

    fn try_as_jtag_probe(&mut self) -> Option<&mut dyn JtagAccess> {
        Some(self)
    }

    fn try_as_dap_probe(&mut self) -> Option<&mut dyn DapProbe> {
        Some(self)
    }
}

impl DapProbe for Xds110 {}

impl RawSwdIo for Xds110 {
    fn swd_io<S>(&mut self, _swdio: S) -> Result<Vec<bool>, DebugProbeError>
    where
        S: IntoIterator<Item = super::IoSequenceItem>,
    {
        Err(DebugProbeError::NotImplemented {
            function_name: "swd_io",
        })
    }

    fn swj_pins(
        &mut self,
        _pin_out: u32,
        _pin_select: u32,
        _pin_wait: u32,
    ) -> Result<u32, DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "swj_pins",
        })
    }

    fn swd_settings(&self) -> &SwdSettings {
        &self.swd_settings
    }

    fn probe_statistics(&mut self) -> &mut super::ProbeStatistics {
        &mut self.probe_statistics
    }
}

// impl RawProtocolIo for Xds110 {
//     /// Shift TMS state. This doesn't actually send any data out, rather it
//     /// updates our own internal state that will be synchronized when we write
//     /// actual data in `jtag_shift_tdi`.
//     fn jtag_shift_tms<M>(&mut self, tms: M, tdi: bool) -> Result<(), DebugProbeError>
//     where
//         M: IntoIterator<Item = bool>,
//     {
//         for tms in tms {
//             self.accumulate_bit(tms, tdi, false)?;
//         }
//         Ok(())
//     }

//     /// Shift bits out. Must be in the correct state.
//     fn jtag_shift_tdi<I>(&mut self, tms: bool, tdi: I) -> Result<(), DebugProbeError>
//     where
//         I: IntoIterator<Item = bool>,
//     {
//         for tdi in tdi {
//             self.accumulate_bit(tms, tdi, false)?;
//         }
//         Ok(())
//     }

//     fn configure_jtag(&mut self, skip_scan: bool) -> Result<(), DebugProbeError> {
//         let ir_lengths = if skip_scan {
//             if let Some(expected_scan_chain) = &self.jtag_state.expected_scan_chain {
//                 self.jtag_state.scan_chain = expected_scan_chain.clone();
//             }
//             self.jtag_state
//                 .expected_scan_chain
//                 .as_ref()
//                 .map(|chain| chain.iter().filter_map(|s| s.ir_len).collect::<Vec<u8>>())
//                 .unwrap_or_default()
//         } else {
//             todo!();
//             // let chain = self.jtag_scan(
//             //     self.jtag_state
//             //         .expected_scan_chain
//             //         .as_ref()
//             //         .map(|chain| {
//             //             chain
//             //                 .iter()
//             //                 .filter_map(|s| s.ir_len)
//             //                 .map(|s| s as usize)
//             //                 .collect::<Vec<usize>>()
//             //         })
//             //         .as_deref(),
//             // )?;
//             // chain.iter().map(|item| item.ir_len()).collect()
//         };
//         tracing::info!("Configuring JTAG with ir lengths: {:?}", ir_lengths);
//         self.select_target(0)?;
//         Ok(())
//     }

//     fn swd_io<S>(&mut self, _swdio: S) -> Result<Vec<bool>, DebugProbeError>
//     where
//         S: IntoIterator<Item = super::arm_debug_interface::IoSequenceItem>,
//     {
//         todo!()
//     }

//     fn swj_pins(
//         &mut self,
//         _pin_out: u32,
//         _pin_select: u32,
//         _pin_wait: u32,
//     ) -> Result<u32, DebugProbeError> {
//         todo!()
//     }

//     fn swd_settings(&self) -> &SwdSettings {
//         &self.swd_settings
//     }

//     fn probe_statistics(&mut self) -> &mut ProbeStatistics {
//         &mut self.probe_statistics
//     }
// }

/// JTAG helper functions
impl Xds110 {
    fn reset_jtag_state_machine(&mut self) -> Result<(), DebugProbeError> {
        // Go to `IDLE` and not `RESET`, because resetting the state machine
        // will put the ICEPICK back into reset.
        self.device.send_command(commands::GotoJtagState {
            state: JtagState::Reset,
            transit: JtagTransit::Quickest,
        })?;
        Ok(())
    }

    fn shift_jtag(
        &mut self,
        register: JtagState,
        data: &[u8],
        mut register_bits: usize,
        capture_data: bool,
    ) -> Result<BitVec, DebugProbeError> {
        let response = self.device.send_command(commands::JtagScan {
            bits: register_bits as u16,
            path: register,
            trans1: JtagTransit::ViaIdle,
            // Note: We would like to go to either `JtagState::Exit1Dr` or `JtagState::Exit1Ir`
            // here to stay in sync with our fake shadow state, but this results in errors
            // from the probe, so we fake it by just going straight to `SelectDr`.
            end_state: JtagState::Dr(RegisterState::Select),
            trans2: JtagTransit::ViaIdle,
            pre: 0,
            post: 0,
            delay: 0,
            rep: 1,
            out_data: data.to_vec(),
            response_length: if capture_data {
                register_bits as u16
            } else {
                0
            },
        })?;
        self.probe_statistics.report_io();
        let mut result = BitVec::new();
        if capture_data {
            for byte in response.data {
                for bit in 0..8 {
                    result.push(byte & 1 << bit != 0);
                    register_bits -= 1;
                    if register_bits == 0 {
                        break;
                    }
                }
                if register_bits == 0 {
                    break;
                }
            }
        }
        Ok(result)
    }

    fn shift_dr(
        &mut self,
        data: &[u8],
        register_bits: usize,
        capture_data: bool,
    ) -> Result<BitVec, DebugProbeError> {
        self.shift_jtag(
            JtagState::Dr(RegisterState::Shift),
            data,
            register_bits,
            capture_data,
        )
    }

    fn shift_ir(
        &mut self,
        data: &[u8],
        register_bits: usize,
        capture_data: bool,
    ) -> Result<BitVec, DebugProbeError> {
        self.shift_jtag(
            JtagState::Ir(RegisterState::Shift),
            data,
            register_bits,
            capture_data,
        )
    }
}

// impl RawDapAccess for Xds110 {
//     fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
//         let mut transfer = DapTransfer::read(address);
//         perform_transfers(self, std::slice::from_mut(&mut transfer))?;

//         match transfer.status {
//             TransferStatus::Ok => Ok(transfer.value),
//             TransferStatus::Failed(DapError::FaultResponse) => {
//                 tracing::debug!("DAP FAULT");

//                 // A fault happened during operation.

//                 // To get a clue about the actual fault we want to read the ctrl register,
//                 // which will have the fault status flags set. But we only do this
//                 // if we are *not* currently reading the ctrl register, otherwise
//                 // this could end up being an endless recursion.

//                 if address == CTRL_PORT {
//                     //  This is not necessarily the CTRL/STAT register, because the dpbanksel field in the SELECT register
//                     //  might be set so that the read wasn't actually from the CTRL/STAT register.
//                     tracing::debug!(
//                         "Read might have been from CTRL/STAT register, not reading it again to dermine fault reason"
//                     );

//                     // We still clear the sticky error, otherwise all future accesses will fail.
//                     //
//                     // We also assume that we use overrun detection, so we clear the overrun error as well.
//                     clear_overrun_and_sticky_err(self)?;
//                 } else {
//                     // Reading the CTRL/AP register depends on the dpbanksel register, but we don't know
//                     // here what the value of it is. So this will fail if dpbanksel is not set to 0,
//                     // but there is no way of figuring that out here, because reading the SELECT register
//                     // would also fail.
//                     //
//                     // What might happen is that the read fails, but that would then trigger another fault handling,
//                     // so it all ends up working.
//                     tracing::debug!("Reading CTRL/AP register to determine reason for FAULT");
//                     let response = RawDapAccess::raw_read_register(self, CTRL_PORT)?;
//                     let ctrl = Ctrl::try_from(response)?;
//                     tracing::debug!(
//                         "Reading DAP register failed. Ctrl/Stat register value is: {:#?}",
//                         ctrl
//                     );

//                     // Check the reason for the fault
//                     // Other fault reasons than overrun or write error are not handled yet.
//                     if ctrl.sticky_orun() || ctrl.sticky_err() {
//                         // Clear the error state
//                         clear_overrun_and_sticky_err(self)?;
//                     }
//                 }

//                 Err(DapError::FaultResponse.into())
//             }
//             // The other errors mean that something went wrong with the protocol itself.
//             // There's no guaranteed correct way to recover, so don't.
//             TransferStatus::Failed(e) => Err(e.into()),
//             other => panic!(
//                 "Unexpected transfer state after reading register: {other:?}. This is a bug!"
//             ),
//         }
//     }

//     fn raw_read_block(
//         &mut self,
//         address: RegisterAddress,
//         values: &mut [u32],
//     ) -> Result<(), ArmError> {
//         let mut transfers = vec![DapTransfer::read(address); values.len()];

//         perform_transfers(self, &mut transfers)?;

//         for (i, result) in transfers.iter().enumerate() {
//             match result.status {
//                 TransferStatus::Ok => values[i] = result.value,
//                 TransferStatus::Failed(err) => {
//                     tracing::info!(
//                         "Error in access {}/{} of block access: {:?}",
//                         i + 1,
//                         values.len(),
//                         err
//                     );

//                     // TODO: The error reason could be investigated by reading the CTRL/STAT register here,

//                     if err == DapError::FaultResponse {
//                         clear_overrun_and_sticky_err(self)?;
//                     }

//                     return Err(err.into());
//                 }
//                 other => panic!(
//                     "Unexpected transfer state after reading registers: {other:?}. This is a bug!"
//                 ),
//             }
//         }

//         Ok(())
//     }

//     fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
//         let mut transfer = DapTransfer::write(address, value);

//         perform_transfers(self, std::slice::from_mut(&mut transfer))?;

//         match transfer.status {
//             TransferStatus::Ok => Ok(()),
//             TransferStatus::Failed(DapError::FaultResponse) => {
//                 tracing::warn!("DAP FAULT");
//                 // A fault happened during operation.

//                 // To get a clue about the actual fault we read the ctrl register,
//                 // which will have the fault status flags set.

//                 // This read might fail because the dpbanksel register is not set to 0.
//                 let response = RawDapAccess::raw_read_register(self, CTRL_PORT)?;

//                 let ctrl = Ctrl::try_from(response)?;
//                 tracing::warn!(
//                     "Writing DAP register failed. Ctrl/Stat register value is: {:#?}",
//                     ctrl
//                 );

//                 // Check the reason for the fault
//                 // Other fault reasons than overrun or write error are not handled yet.
//                 if ctrl.sticky_orun() || ctrl.sticky_err() {
//                     // We did not handle a WAIT state properly

//                     // Because we use overrun detection, we now have to clear the overrun error
//                     clear_overrun_and_sticky_err(self)?;
//                 }

//                 Err(DapError::FaultResponse.into())
//             }
//             // The other errors mean that something went wrong with the protocol itself.
//             // There's no guaranteed correct way to recover, so don't.
//             TransferStatus::Failed(e) => Err(e.into()),
//             other => panic!(
//                 "Unexpected transfer state after writing register: {other:?}. This is a bug!"
//             ),
//         }
//     }

//     fn raw_write_block(
//         &mut self,
//         address: RegisterAddress,
//         values: &[u32],
//     ) -> Result<(), ArmError> {
//         let mut transfers = values
//             .iter()
//             .map(|v| DapTransfer::write(address, *v))
//             .collect::<Vec<_>>();

//         perform_transfers(self, &mut transfers)?;

//         for (i, result) in transfers.iter().enumerate() {
//             match result.status {
//                 TransferStatus::Ok => {}
//                 TransferStatus::Failed(err) => {
//                     tracing::debug!(
//                         "Error in access {}/{} of block access: {}",
//                         i + 1,
//                         values.len(),
//                         err
//                     );

//                     // TODO: The error reason could be investigated by reading the CTRL/STAT register here,
//                     if err == DapError::FaultResponse {
//                         clear_overrun_and_sticky_err(self)?;
//                     }

//                     return Err(err.into());
//                 }
//                 other => panic!(
//                     "Unexpected transfer state after writing registers: {other:?}. This is a bug!"
//                 ),
//             }
//         }

//         Ok(())
//     }

//     fn jtag_sequence(&mut self, cycles: u8, tms: bool, tdi: u64) -> Result<(), DebugProbeError> {
//         for bit in 0..cycles {
//             self.accumulate_bit(tms, tdi & 1 << bit != 0, false)?
//         }
//         Ok(())
//     }

//     fn swj_sequence(&mut self, _bit_len: u8, _bits: u64) -> Result<(), DebugProbeError> {
//         todo!()
//     }

//     fn swj_pins(
//         &mut self,
//         _pin_out: u32,
//         _pin_select: u32,
//         _pin_wait: u32,
//     ) -> Result<u32, DebugProbeError> {
//         todo!()
//     }

//     fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
//         self
//     }

//     fn core_status_notification(
//         &mut self,
//         _state: crate::CoreStatus,
//     ) -> Result<(), DebugProbeError> {
//         Ok(())
//     }
// }

impl Xds110 {
    fn make_out_data(
        &self,
        data: &[u8],
        pre_bits: usize,
        post_bits: usize,
        length: usize,
    ) -> BitVec<u8, Lsb0> {
        std::iter::repeat_n(true, pre_bits)
            .chain(data.as_bits::<Lsb0>()[..length].iter().map(|b| *b))
            .chain(std::iter::repeat_n(true, post_bits))
            .collect()
    }
}

impl JtagAccess for Xds110 {
    fn set_scan_chain(&mut self, scan_chain: &[ScanChainElement]) -> Result<(), DebugProbeError> {
        tracing::trace!("Setting scan chain to: {:?}", scan_chain);
        self.jtag_state.expected_scan_chain = Some(scan_chain.to_vec());
        self.jtag_state.scan_chain = scan_chain.to_vec();
        Ok(())
    }

    fn scan_chain(&mut self) -> Result<&[ScanChainElement], DebugProbeError> {
        if !self.jtag_state.scan_chain.is_empty() {
            tracing::trace!("Scan chain already exists! Returning existing scan chain");
            return Ok(&self.jtag_state.scan_chain);
        }

        const MAX_CHAIN: usize = 8;

        self.reset_jtag_state_machine()?;

        self.jtag_state.chain_params = ChainParams::default();

        let input = [0xFF; 4 * MAX_CHAIN];

        let response = self.shift_dr(&input, input.len() * 8, true)?;

        tracing::debug!("DR: {:?}", response);

        let idcodes = extract_idcodes(&response)?;

        tracing::info!(
            "JTAG DR scan complete, found {} TAPs. {:?}",
            idcodes.len(),
            idcodes
        );

        tracing::debug!("Scanning JTAG chain for IR lengths");

        // First shift out all ones
        let input = vec![0xff; idcodes.len()];
        let response = self.shift_ir(&input, input.len() * 8, true)?;

        tracing::debug!("IR scan: {}", response);

        self.reset_jtag_state_machine()?;

        // Next, shift out same amount of zeros, then ones to make sure the IRs contain BYPASS.
        let input = std::iter::repeat_n(0, idcodes.len())
            .chain(input.iter().copied())
            .collect::<Vec<_>>();
        let response_zeros = self.shift_ir(&input, input.len() * 8, true)?;

        tracing::debug!("IR scan: {}", response_zeros);

        let response = response.as_bitslice();
        let response = common_sequence(response, response_zeros.as_bitslice());

        tracing::debug!("IR scan: {}", response);

        let ir_lens = extract_ir_lengths(
            response,
            idcodes.len(),
            self.jtag_state
                .expected_scan_chain
                .as_ref()
                .map(|chain| {
                    chain
                        .iter()
                        .filter_map(|s| s.ir_len)
                        .map(|s| s as usize)
                        .collect::<Vec<usize>>()
                })
                .as_deref(),
        )?;

        tracing::info!("Found {} TAPs on reset scan", idcodes.len());
        tracing::debug!("Detected IR lens: {:?}", ir_lens);

        let chain = idcodes
            .into_iter()
            .zip(ir_lens)
            .map(|(idcode, irlen)| ScanChainElement {
                ir_len: Some(irlen as u8),
                name: idcode.map(|i| i.to_string()),
            })
            .collect::<Vec<_>>();

        self.jtag_state.scan_chain = chain;

        // Select target 0 by default
        self.select_target(0)?;

        Ok(self.jtag_state.scan_chain.as_slice())
    }

    fn tap_reset(&mut self) -> Result<(), DebugProbeError> {
        tracing::trace!("Resetting TAP");
        Ok(())
    }

    /// Configures the probe to address the given target.
    fn select_target(&mut self, target: usize) -> Result<(), DebugProbeError> {
        if self.jtag_state.scan_chain.is_empty() {
            self.scan_chain()?;
        }

        let Some(params) = ChainParams::from_jtag_chain(&self.jtag_state.scan_chain, target) else {
            return Err(DebugProbeError::TargetNotFound);
        };

        let max_ir_address = (1 << params.irlen) - 1;

        tracing::debug!("Selecting JTAG TAP: {target}");
        tracing::debug!("Setting chain params: {params:?}");
        tracing::debug!("Setting max_ir_address to {max_ir_address}");

        self.jtag_state.chain_params = params;
        assert_eq!(self.jtag_state.max_ir_address(), max_ir_address);
        self.current_ir = None;

        Ok(())
    }

    fn set_idle_cycles(&mut self, idle_cycles: u8) -> Result<(), DebugProbeError> {
        tracing::trace!("Setting idle cycles to: {}", idle_cycles);
        self.idle_cycles = idle_cycles;
        Ok(())
    }

    fn idle_cycles(&self) -> u8 {
        self.idle_cycles
    }

    fn write_register(
        &mut self,
        address: u32,
        data: &[u8],
        len: u32,
    ) -> Result<BitVec, DebugProbeError> {
        tracing::trace!(
            "Writing to register {}. Current chain params: {:?}",
            address,
            self.jtag_state.chain_params
        );

        if self.current_ir != Some(address) {
            tracing::trace!("Selecting IR {}", address);
            let address_data = self.make_out_data(
                &address.to_le_bytes(),
                self.jtag_state.chain_params.irpre,
                self.jtag_state.chain_params.irpost,
                self.jtag_state.chain_params.irlen,
            );
            let address_len = address_data.len() as u16;
            let out_data = address_data.into_vec();
            let out_data_len = out_data.len() as u16;
            // Note: The `pre` and `post` values don't appear to actually work. So we need to pack
            // and unpack the pre and post bits ourselves.
            let result = self.device.send_command(commands::JtagScan {
                bits: address_len,
                path: JtagState::Ir(RegisterState::Shift),
                trans1: JtagTransit::ViaIdle,
                // Note: We would like to go to either `JtagState::Exit1Dr` or `JtagState::Exit1Ir`
                // here to stay in sync with our fake shadow state, but this results in errors
                // from the probe, so we fake it by just going straight to `SelectDr`.
                end_state: JtagState::Idle,
                trans2: JtagTransit::ViaIdle,
                pre: 0,
                post: 0,
                delay: 0,
                rep: 1,
                out_data,
                response_length: out_data_len,
            })?;
            self.probe_statistics.report_io();
            self.current_ir = Some(address);
            tracing::trace!("Selected IR and got response: {:?}", result);
        }

        let out_data = self.make_out_data(
            data,
            self.jtag_state.chain_params.drpre,
            self.jtag_state.chain_params.drpost,
            len as usize,
        );
        let bits = out_data.len() as u16;
        let out_data = out_data.into_vec();
        let response_length = out_data.len() as u16;

        let result = self.device.send_command(commands::JtagScan {
            bits,
            path: JtagState::Dr(RegisterState::Shift),
            trans1: JtagTransit::ViaIdle,
            // Note: We would like to go to either `JtagState::Exit1Dr` or `JtagState::Exit1Ir`
            // here to stay in sync with our fake shadow state, but this results in errors
            // from the probe, so we fake it by just going straight to `SelectDr`.
            end_state: JtagState::Idle,
            trans2: JtagTransit::ViaIdle,
            pre: 0,
            post: 0,
            delay: 0,
            rep: 1,
            out_data,
            response_length,
        })?;
        self.probe_statistics.report_io();
        tracing::trace!("Result from jtag scan: {:x?}", result);

        let ret_bytes = result.data;
        let mut ret = bitvec![];
        for byte in ret_bytes.iter() {
            ret.extend_from_bitslice(byte.view_bits::<Lsb0>());
        }
        // std::process::exit(0);
        Ok(ret)
    }

    fn write_dr(&mut self, _data: &[u8], _len: u32) -> Result<BitVec, DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "write_dr",
        })
    }

    fn shift_raw_sequence(
        &mut self,
        sequence: super::JtagSequence,
    ) -> Result<BitVec, DebugProbeError> {
        let tms = std::iter::repeat(sequence.tms);
        let tdi = sequence.data.into_iter();
        let cap = std::iter::repeat(sequence.tdo_capture);

        for ((tms, tdi), cap) in tms.into_iter().zip(tdi.into_iter()).zip(cap.into_iter()) {
            self.accumulate_bit(tms, tdi, cap)?;
            self.jtag_state.state.update(tms);
        }
        Ok(std::mem::take(&mut self.response))
    }

    fn configure_jtag(&mut self, skip_scan: bool) -> Result<(), DebugProbeError> {
        let ir_lengths = if skip_scan {
            if let Some(expected_scan_chain) = &self.jtag_state.expected_scan_chain {
                self.jtag_state.scan_chain = expected_scan_chain.clone();
            }
            self.jtag_state
                .expected_scan_chain
                .as_ref()
                .map(|chain| chain.iter().filter_map(|s| s.ir_len).collect::<Vec<u8>>())
                .unwrap_or_default()
        } else {
            return Err(DebugProbeError::NotImplemented {
                function_name: "configure_jtag(false)",
            });
        };
        tracing::info!("Configuring JTAG with ir lengths: {:?}", ir_lengths);
        self.select_target(0)?;
        Ok(())
    }
}

// impl RawJtagIo for Xds110 {
//     fn shift_bit(
//         &mut self,
//         tms: bool,
//         tdi: bool,
//         capture_tdo: bool,
//     ) -> Result<(), DebugProbeError> {
//         self.accumulate_bit(tms, tdi, capture_tdo)?;
//         self.state_mut().state.update(tms);
//         Ok(())
//     }

//     fn read_captured_bits(&mut self) -> Result<BitVec, DebugProbeError> {
//         tracing::trace!("Returning captured bits: {:?}", self.response);
//         Ok(std::mem::take(&mut self.response))
//     }

//     fn state_mut(&mut self) -> &mut JtagDriverState {
//         &mut self.jtag_state
//     }

//     fn state(&self) -> &JtagDriverState {
//         &self.jtag_state
//     }

//     fn reset_jtag_state_machine(&mut self) -> Result<(), DebugProbeError> {
//         tracing::error!("Ignoring reset request");
//         Ok(())
//     }

//     fn configure_jtag(&mut self, skip_scan: bool) -> Result<(), DebugProbeError> {
//         let ir_lengths = if skip_scan {
//             if let Some(expected_scan_chain) = &self.jtag_state.expected_scan_chain {
//                 self.jtag_state.scan_chain = expected_scan_chain.clone();
//             }
//             self.jtag_state
//                 .expected_scan_chain
//                 .as_ref()
//                 .map(|chain| chain.iter().filter_map(|s| s.ir_len).collect::<Vec<u8>>())
//                 .unwrap_or_default()
//         } else {
//             return Err(DebugProbeError::NotImplemented {
//                 function_name: "configure_jtag(false)",
//             });
//         };
//         tracing::info!("Configuring JTAG with ir lengths: {:?}", ir_lengths);
//         self.select_target(0)?;
//         Ok(())
//     }
// }

fn get_xds110_info(device: &DeviceInfo) -> Option<DebugProbeInfo> {
    for xds110 in XDS110_USB_DEVICES {
        if xds110.vid == device.vendor_id() && xds110.pid == device.product_id() {
            return Some(DebugProbeInfo::new(
                device.product_string().unwrap_or("XDS110").to_string(),
                device.vendor_id(),
                device.product_id(),
                // Ensure the serial number always exists, and ends with the native suffix
                device
                    .serial_number()
                    .map(|s| format!("{}{}", s, USB_SERIAL_NATIVE_SUFFIX))
                    .or_else(|| Some(USB_SERIAL_NATIVE_SUFFIX.to_owned())),
                &Xds110Factory,
                None,
            ));
        }
    }
    None
}

#[tracing::instrument(skip_all)]
fn list_xds110_devices() -> Vec<DebugProbeInfo> {
    tracing::debug!("Searching for XDS110 probes");
    let Ok(devices) = nusb::list_devices().wait() else {
        return vec![];
    };
    let mut probes = vec![];
    for device in devices {
        if let Some(info) = get_xds110_info(&device) {
            probes.push(info);
        }
    }

    tracing::debug!("Found {} total XDS110 probes", probes.len());
    probes
}

#[derive(thiserror::Error, Debug, docsplay::Display)]
pub(crate) enum Xds110Error {
    /// Not enough bytes written.
    NotEnoughBytesWritten { is: usize, should: usize },

    /// Magic value of `*` was not found.
    MagicValueNotFound,

    /// Not enough bytes read.
    NotEnoughBytesRead { is: usize, should: usize },

    /// Usb configuration not found.
    ConfigurationNotFound,

    /// Usb interface not found.
    InterfaceNotFound,

    /// Usb endpoint not found.
    EndpointNotFound,

    /// Invalid payload.
    InvalidPayload,
}

impl ProbeError for Xds110Error {}
