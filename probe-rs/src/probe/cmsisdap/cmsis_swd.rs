use std::collections::VecDeque;
use std::time::Duration;

use crate::{
    architecture::arm::{
        RegisterAddress,
        dp::{Abort, Ctrl, DpRegister, DpRegisterAddress},
    },
    probe::{
        BatchError, BatchExecutionError, BitSequence, CommandResult, DebugProbeError, HandleId,
        Results, WireProtocol,
        cmsisdap::{
            CmsisDap,
            commands::{
                self,
                swj::sequence::SequenceRequest,
                transfer::{
                    Ack, RW, TransferBlockRequest, TransferBlockResponse, TransferRequest,
                    TransferResponse,
                },
            },
        },
        swd::{
            Direction, Pins as SwdPins, Port, SwdBatch, SwdOp, SwdProbe, SwdTransferError, is_abort,
        },
    },
};

pub(crate) fn cmsis_handles_wait(wait_retry: u16) -> bool {
    wait_retry > 0
}

/// How often an access repeats before `DAP_TransferBlock` carries it.
///
/// `DAP_TransferBlock` sends the access once for every repeat, so it needs
/// fewer packets than `DAP_Transfer`.
const MIN_BLOCK_TRANSFERS: usize = 2;

/// How many `DAP_TransferBlock` requests may be outstanding at once.
///
/// Overlapping a request with the previous reply hides one round trip each time. Past a handful
/// the wait is already covered and a deeper queue only holds more requests in memory.
const MAX_PIPELINED_BLOCKS: usize = 8;

/// Return how many words one `DAP_TransferBlock` packet holds.
pub(crate) fn block_words_per_packet(packet_size: u16) -> usize {
    // A packet holds a one byte HID report id, the command id, the DAP index,
    // two length bytes, and the request byte, before the data.
    ((packet_size as usize - 6) / 4).max(1)
}

fn transfer_data(op: &SwdOp) -> u32 {
    match op {
        SwdOp::Transfer { data, .. } => *data,
        _ => 0,
    }
}

/// Return how many operations from `start` repeat the same access.
pub(crate) fn run_length(ops: &[(HandleId, SwdOp)], start: usize) -> usize {
    let SwdOp::Transfer {
        port,
        addr,
        direction,
        ..
    } = ops[start].1
    else {
        return 0;
    };

    ops[start..]
        .iter()
        .take_while(|(_, op)| {
            matches!(
                op,
                SwdOp::Transfer {
                    port: run_port,
                    addr: run_addr,
                    direction: run_direction,
                    ..
                } if *run_port == port && *run_addr == addr && *run_direction == direction
            )
        })
        .count()
}

/// Over JTAG, the span of a batch that a sticky-error check has to cover: the index of the first
/// AP transfer, which is blamed if an error is found, and of the last transfer, after which
/// CTRL/STAT is read. None when there is nothing to check.
///
/// A JTAG-DP acknowledges OK and FAULT with one value, and `DAP_Transfer` passes it on as OK: the
/// CMSIS-DAP specification defines ACK 1 as "OK (for SWD protocol), OK or FAULT (for JTAG
/// protocol)". So an AP transfer the DP refused looks like one that went through, and leaves
/// CTRL/STAT.STICKYERR set, after which the DP refuses every AP access. The only way to notice
/// is to read CTRL/STAT afterwards, as the raw JTAG path does after each of its transfer sets.
///
/// Only an AP transfer sets the flag, so a batch without one is not checked. Nor is a batch whose
/// last transfer writes ABORT: that clears the flags, so a read after it would say nothing.
fn transfers_to_check(ops: &[(HandleId, SwdOp)]) -> Option<(usize, usize)> {
    let first_ap = ops
        .iter()
        .position(|(_, op)| matches!(op, SwdOp::Transfer { port: Port::Ap, .. }))?;
    let last = ops
        .iter()
        .rposition(|(_, op)| matches!(op, SwdOp::Transfer { .. }))?;
    (!is_abort(&ops[last].1)).then_some((first_ap, last))
}

/// Split a run of transfers into the packets that carry them.
///
/// Each packet arrives with its request built and its transfers carrying the batch index a fault
/// is reported against. `run_start` is where the run sits in the batch.
///
/// With `abort_alone`, a write to ABORT also ends the run.
fn split_into_packets(
    ops: &[(HandleId, SwdOp)],
    run_start: usize,
    dap_index: u8,
    packet_size: u16,
    abort_alone: bool,
) -> impl Iterator<Item = PendingBatch> {
    let mut offset = 0;

    std::iter::from_fn(move || {
        let mut packet = PendingBatch::new(dap_index);

        while let Some((_, op)) = ops.get(offset) {
            if abort_alone && is_abort(op) {
                break;
            }
            let SwdOp::Transfer { direction, .. } = op else {
                break;
            };

            // A repeat of one access can go as a block transfer, which is pipelined and, for a
            // write, denser -- but only if it gets a packet to itself. Ask for the whole run: if
            // it fits here, whatever follows can still share the packet; if it does not, the
            // block command takes it.
            let run = run_length(ops, offset);
            let wanted = if run >= MIN_BLOCK_TRANSFERS { run } else { 1 };

            let run_items =
                ops[offset..offset + wanted]
                    .iter()
                    .enumerate()
                    .map(|(index, (id, op))| {
                        let SwdOp::Transfer {
                            port, addr, data, ..
                        } = op
                        else {
                            unreachable!("a run holds only transfers")
                        };
                        let address = CmsisDap::swd_register(*port, *addr);
                        (id.clone(), address, *data, run_start + offset + index)
                    });

            if !packet.push_all(*direction, packet_size, run_items) {
                break;
            }
            offset += wanted;
        }

        (!packet.is_empty()).then_some(packet)
    })
}

struct PendingTransfer {
    id: HandleId,
    direction: Direction,
    batch_index: usize,
}

/// Transfers queued for one `DAP_Transfer`, and what its reply maps back to.
///
/// The two halves stay in step: entry `n` of `transfers` describes transfer `n` of `request`, which
/// is what lets a reply be matched to the batch operation that asked for it. Only
/// [`Self::push_all`] adds to either, except for the CTRL/STAT read that [`Self::check_status`]
/// puts last in `request`, which answers to no batch operation.
struct PendingBatch {
    request: TransferRequest,
    transfers: Vec<PendingTransfer>,
    /// Whether `request` ends in a read of CTRL/STAT that looks for a sticky error, and the batch
    /// index to blame if it finds one.
    status_check: Option<usize>,
}

impl PendingBatch {
    fn new(dap_index: u8) -> Self {
        let mut request = TransferRequest::empty();
        request.dap_index = dap_index;
        Self {
            request,
            transfers: Vec::new(),
            status_check: None,
        }
    }

    fn is_empty(&self) -> bool {
        self.transfers.is_empty() && self.status_check.is_none()
    }

    /// End the packet with a read of CTRL/STAT, so that its reply tells whether a transfer was
    /// refused -- see [`transfers_to_check`]. `blame` is the batch index a sticky error is
    /// reported against. Returns whether the packet had room; an empty one always has.
    fn check_status(&mut self, blame: usize, packet_size: u16) -> bool {
        if self.status_check.is_some() {
            return true;
        }
        if !self.is_empty() && !self.request.has_room_for(RW::R, packet_size) {
            return false;
        }
        self.request.add_read(Ctrl::ADDRESS.into());
        self.status_check = Some(blame);
        true
    }

    /// Push every transfer `items` yields, or none of them when the packet runs out of room.
    ///
    /// Returns whether they all fit. The whole run is rewound on the first one that does not, so a
    /// run either travels in this packet or waits for the next one intact.
    ///
    /// An empty packet always takes one transfer however small the packet is, or splitting a batch
    /// would never make progress.
    fn push_all(
        &mut self,
        direction: Direction,
        packet_size: u16,
        items: impl IntoIterator<Item = (HandleId, RegisterAddress, u32, usize)>,
    ) -> bool {
        let mark = self.request.mark();
        let transfers = self.transfers.len();

        for (id, address, data, batch_index) in items {
            if !self.request.has_room_for(Self::rw(direction), packet_size) && !self.is_empty() {
                self.request.rewind(mark);
                self.transfers.truncate(transfers);
                return false;
            }

            match direction {
                Direction::Read => self.request.add_read(address),
                Direction::Write => self.request.add_write(address, data),
            }
            self.transfers.push(PendingTransfer {
                id,
                direction,
                batch_index,
            });
        }
        true
    }

    fn rw(direction: Direction) -> RW {
        match direction {
            Direction::Read => RW::R,
            Direction::Write => RW::W,
        }
    }
}

impl CmsisDap {
    fn swd_register(port: Port, addr: u8) -> RegisterAddress {
        match port {
            Port::Dp => RegisterAddress::DpRegister(DpRegisterAddress {
                address: addr,
                bank: None,
            }),
            Port::Ap => RegisterAddress::ApRegister(addr),
        }
    }

    fn ack_to_transfer_error(ack: Ack) -> SwdTransferError {
        match ack {
            Ack::Ok => SwdTransferError::Protocol,
            Ack::Wait => SwdTransferError::WaitResponse,
            Ack::Fault => SwdTransferError::FaultResponse,
            Ack::NoAck => SwdTransferError::NoAcknowledge,
        }
    }

    /// Classify one `DAP_Transfer` reply.
    ///
    /// Pure, like [`Self::classify_block_response`], so recovery can wait until the reply has
    /// been taken. Returns the batch index of the transfer the reply blames. With a
    /// `status_check`, the request ended in a CTRL/STAT read of its own, so the reply holds one
    /// transfer more than `transfers`, and the check's index is blamed when there is no transfer
    /// to blame.
    ///
    /// The acknowledgement is classified before the transfer count. A FAULT or WAIT stops the
    /// probe early, so a short count is the *consequence* of that error, not evidence that the
    /// probe failed to report one. Checking the count first hides the acknowledgement and skips
    /// recovery, leaving CTRL/STAT.STICKYERR set - after which the DP faults every subsequent
    /// AP access (IHI0031G B4.2.4) and the session cannot recover on its own.
    fn classify_transfer_response(
        response: &TransferResponse,
        transfers: &[PendingTransfer],
        status_check: Option<usize>,
    ) -> Result<(), (BatchError<DebugProbeError>, usize)> {
        let count = response.transfers.len();
        let expected = transfers.len() + usize::from(status_check.is_some());
        let fault_operation = transfers
            .get(count.min(transfers.len()).saturating_sub(1))
            .map_or(status_check.unwrap_or(0), |transfer| transfer.batch_index);

        if response.last_transfer_response.protocol_error {
            return Err((
                BatchError::Specific(DebugProbeError::SwdTransfer(SwdTransferError::Protocol)),
                fault_operation,
            ));
        }

        match response.last_transfer_response.ack {
            Ack::Ok => {}
            ack => {
                return Err((
                    BatchError::Specific(DebugProbeError::SwdTransfer(
                        Self::ack_to_transfer_error(ack),
                    )),
                    fault_operation,
                ));
            }
        }

        // An OK acknowledgement with a short count really is the probe not saying why.
        if count < expected {
            return Err((
                BatchError::Probe(DebugProbeError::Other(format!(
                    "Possible error in CMSIS-DAP probe: Only {count}/{expected} transfers were executed, but no error was reported.",
                ))),
                fault_operation,
            ));
        }

        Ok(())
    }

    fn flush_pending_transfers(
        &mut self,
        pending: &PendingBatch,
        mut results: Results,
    ) -> Result<Results, BatchExecutionError<DebugProbeError>> {
        if pending.is_empty() {
            return Ok(results);
        }

        let transfers = &pending.transfers;
        let response = match commands::send_command(&mut self.device, &pending.request) {
            Ok(response) => response,
            Err(error) => {
                let fault_operation = transfers[0].batch_index;
                return Err(BatchExecutionError::new_from_debug_probe_at(
                    DebugProbeError::from(error),
                    results,
                    fault_operation,
                ));
            }
        };

        if let Err((error, fault_operation)) =
            Self::classify_transfer_response(&response, transfers, pending.status_check)
        {
            self.recover_from_block_error(&error);
            return Err(BatchExecutionError {
                error,
                results,
                fault_operation,
            });
        }

        if let Some(blame) = pending.status_check {
            let Some(ctrl) = response.transfers.last().and_then(|transfer| transfer.data) else {
                return Err(BatchExecutionError::new_from_debug_probe_at(
                    DebugProbeError::Other(
                        "CMSIS-DAP read of CTRL/STAT did not return any data".to_string(),
                    ),
                    results,
                    blame,
                ));
            };
            if Ctrl(ctrl).sticky_err() {
                // An AP transfer was refused: one of this batch, or of an earlier one if nothing
                // has read CTRL/STAT since. Which one is unknown, and the DP has discarded every
                // AP access since, so none of the results stands.
                tracing::debug!(
                    "CTRL/STAT reports a sticky error after a JTAG batch: {ctrl:#010x}"
                );
                self.clear_sticky_err();
                return Err(BatchExecutionError {
                    error: BatchError::Specific(DebugProbeError::SwdTransfer(
                        SwdTransferError::FaultResponse,
                    )),
                    results: Results::new(),
                    fault_operation: blame,
                });
            }
        }

        for (transfer, response_transfer) in transfers.iter().zip(response.transfers.iter()) {
            if transfer.direction != Direction::Read || !transfer.id.should_capture() {
                continue;
            }
            let Some(data) = response_transfer.data else {
                return Err(BatchExecutionError::new_from_debug_probe_at(
                    DebugProbeError::Other("CMSIS-DAP read did not return any data".to_string()),
                    results,
                    transfer.batch_index,
                ));
            };
            results.push(&transfer.id, CommandResult::U32(data));
        }
        Ok(results)
    }

    fn max_words_per_block_packet(&self) -> usize {
        block_words_per_packet(self.packet_size)
    }

    /// Classify one `DAP_TransferBlock` reply.
    ///
    /// Takes no `self`, so it cannot issue a command. Whatever the reply says, recovery has to
    /// wait until every outstanding reply has been taken.
    fn classify_block_response(
        response: &TransferBlockResponse,
        chunk_len: usize,
        chunk_start: usize,
    ) -> Result<(), (BatchError<DebugProbeError>, usize)> {
        let count = usize::from(response.transfer_count);
        let fault_operation = chunk_start + count.min(chunk_len).saturating_sub(1);

        if response.transfer_response.protocol_error {
            return Err((
                BatchError::Specific(DebugProbeError::SwdTransfer(SwdTransferError::Protocol)),
                fault_operation,
            ));
        }

        match response.transfer_response.ack {
            Ack::Ok => {}
            ack => {
                return Err((
                    BatchError::Specific(DebugProbeError::SwdTransfer(
                        Self::ack_to_transfer_error(ack),
                    )),
                    fault_operation,
                ));
            }
        }

        if count < chunk_len {
            return Err((
                BatchError::Probe(DebugProbeError::Other(format!(
                    "Possible error in CMSIS-DAP probe: Only {}/{} transfers were executed, but no error was reported.",
                    count, chunk_len
                ))),
                fault_operation,
            ));
        }

        Ok(())
    }

    /// Clear CTRL/STAT.STICKYERR, which a status check found set. Until it is cleared the DP
    /// refuses every AP access.
    fn clear_sticky_err(&mut self) {
        let mut abort = Abort(0);
        abort.set_stkerrclr(true);
        if let Err(error) = self.write_abort(abort) {
            tracing::warn!("Failed to clear the sticky error: {error}");
        }
    }

    /// Put the port back in order after a block transfer failed.
    ///
    /// Each arm is a command in its own right, which is why this is separate from classifying the
    /// reply: issuing one while the probe still owes replies leaves the two out of step.
    fn recover_from_block_error(&mut self, error: &BatchError<DebugProbeError>) {
        match error {
            BatchError::Specific(DebugProbeError::SwdTransfer(SwdTransferError::FaultResponse)) => {
                if let Err(error) = self.handle_sticky_err() {
                    tracing::warn!("Failed to clear the sticky error: {error}");
                }
            }
            BatchError::Specific(DebugProbeError::SwdTransfer(SwdTransferError::WaitResponse)) => {
                let abort = {
                    let mut abort = Abort(0);
                    abort.set_dapabort(true);
                    abort
                };
                if let Err(error) = self.write_abort(abort) {
                    tracing::warn!("Failed to abort the transfer: {error}");
                }
            }
            _ => {}
        }
    }

    /// Send a repeated access as `DAP_TransferBlock` packets, several at a time.
    ///
    /// Sending the next request before reading the previous reply hides one round trip per
    /// overlap. ADIv5 §B4.2.4 describes this as the intended use: "A debugger might stream a block
    /// of data and then check the CTRL/STAT register at the end of the block."
    ///
    /// A request already on the wire when an earlier one faults cannot reach the target. Once a
    /// sticky flag is set the DP answers FAULT to every access except reads of DPIDR and
    /// CTRL/STAT and writes to ABORT, so the tail is refused rather than performed, and the first
    /// failure in batch order is the one reported.
    fn run_transfer_block(
        &mut self,
        run: &[(HandleId, SwdOp)],
        run_start: usize,
        mut results: Results,
    ) -> Result<Results, BatchExecutionError<DebugProbeError>> {
        let SwdOp::Transfer {
            port,
            addr,
            direction,
            ..
        } = run[0].1
        else {
            return Ok(results);
        };
        let address = Self::swd_register(port, addr);
        let words_per_packet = self.max_words_per_block_packet();
        // Leave one packet buffer free. A probe with all buffers full does not take
        // `DAP_TransferAbort` before the current transfer ends.
        let depth = (self.packet_count as usize)
            .saturating_sub(1)
            .clamp(1, MAX_PIPELINED_BLOCKS);

        let mut chunks = run.chunks(words_per_packet).enumerate();
        let mut in_flight = VecDeque::with_capacity(depth);
        let mut failure: Option<(BatchError<DebugProbeError>, usize)> = None;
        // A chunk that failed ends the run, so nothing after it is the caller's to see. A chunk
        // that was never sent does not: whatever is already in flight ran before the failure and
        // would have run without the pipeline too.
        let mut capture = true;

        loop {
            while failure.is_none() && in_flight.len() < depth {
                let Some((chunk_index, chunk)) = chunks.next() else {
                    break;
                };
                let chunk_start = run_start + chunk_index * words_per_packet;

                let mut request = match direction {
                    Direction::Read => {
                        TransferBlockRequest::read_request(address, chunk.len() as u16)
                    }
                    Direction::Write => TransferBlockRequest::write_request(
                        address,
                        chunk.iter().map(|(_, op)| transfer_data(op)).collect(),
                    ),
                };
                request.dap_index = self.jtag_state.chain_params.index as u8;

                if let Err(error) = commands::send_request(&mut self.device, &request) {
                    failure = Some((BatchError::Probe(DebugProbeError::from(error)), chunk_start));
                    break;
                }

                in_flight.push_back((chunk_start, chunk, request));
            }

            let Some((chunk_start, chunk, request)) = in_flight.pop_front() else {
                break;
            };

            let response = match commands::receive_response(&mut self.device, &request) {
                Ok(response) => response,
                Err(error) => {
                    // A late reply arrives ahead of the replies still in flight, so reading on
                    // would give each request the reply of the one before it.
                    if commands::may_be_out_of_step(&error) || !in_flight.is_empty() {
                        self.device.resynchronise();
                    }
                    failure.get_or_insert((
                        BatchError::Probe(DebugProbeError::from(error)),
                        chunk_start,
                    ));
                    break;
                }
            };

            match Self::classify_block_response(&response, chunk.len(), chunk_start) {
                Err(problem) => {
                    failure.get_or_insert(problem);
                    capture = false;
                }
                Ok(()) if !capture => {}
                Ok(()) => {
                    if direction == Direction::Read {
                        for ((id, _), value) in chunk.iter().zip(response.transfer_data.iter()) {
                            if id.should_capture() {
                                results.push(id, CommandResult::U32(*value));
                            }
                        }
                    }
                }
            }
        }

        match failure {
            Some((error, fault_operation)) => {
                self.recover_from_block_error(&error);
                Err(BatchExecutionError {
                    error,
                    results,
                    fault_operation,
                })
            }
            None => Ok(results),
        }
    }

    fn run_swj_sequence_op(&mut self, bits: &BitSequence) -> Result<(), DebugProbeError> {
        self.connect_if_needed()?;

        const MAX_BITS: usize = 256;
        let mut offset = 0;
        while offset < bits.len() {
            let chunk_len = (bits.len() - offset).min(MAX_BITS);
            let mut data = vec![0u8; chunk_len.div_ceil(8)];
            for i in 0..chunk_len {
                if bits[offset + i] {
                    data[i / 8] |= 1 << (i % 8);
                }
            }
            let bit_count = if chunk_len == MAX_BITS {
                0
            } else {
                chunk_len as u8
            };
            self.send_swj_sequences(SequenceRequest::new(&data, bit_count)?)
                .map_err(DebugProbeError::from)?;
            offset += chunk_len;
        }

        Ok(())
    }

    fn run_swj_idle(&mut self, cycles: u32) -> Result<(), DebugProbeError> {
        let mut remaining = cycles as usize;
        while remaining > 0 {
            let chunk_len = remaining.min(256);
            let data = vec![0u8; chunk_len.div_ceil(8)];
            let bit_count = if chunk_len == 256 { 0 } else { chunk_len as u8 };
            self.send_swj_sequences(SequenceRequest::new(&data, bit_count)?)
                .map_err(DebugProbeError::from)?;
            remaining -= chunk_len;
        }
        Ok(())
    }

    fn run_swj_pins_op(
        &mut self,
        out: SwdPins,
        select: SwdPins,
        wait: Duration,
    ) -> Result<SwdPins, DebugProbeError> {
        self.connect_if_needed()?;

        let request = commands::swj::pins::SWJPinsRequest::from_raw_values(
            out.0,
            select.0,
            wait.as_micros() as u32,
        );
        let levels = commands::send_command(&mut self.device, &request)?;
        Ok(SwdPins(levels.0))
    }
}

impl SwdProbe for CmsisDap {
    fn run_batch(
        &mut self,
        batch: &SwdBatch,
    ) -> Result<Results, BatchExecutionError<DebugProbeError>> {
        let mut results = Results::new();
        let ops: Vec<(HandleId, SwdOp)> = batch
            .iter()
            .map(|(id, op)| (id.clone(), op.clone()))
            .collect();
        let dap_index = self.jtag_state.chain_params.index as u8;

        // Over JTAG the firmware performs these transfers on the JTAG-DP, starting from
        // Run-Test/Idle, and ABORT is not a register that can be reached.
        let jtag = self.protocol == Some(WireProtocol::Jtag);
        if jtag && let Err(error) = self.enter_run_test_idle() {
            return Err(BatchExecutionError::new_from_debug_probe_at(
                error, results, 0,
            ));
        }
        // Nor does it report a FAULT, so CTRL/STAT is read once the transfers are done: in the
        // packet that carries the last of them when it has room, else in one of its own.
        let status_check = if jtag { transfers_to_check(&ops) } else { None };
        let mut checked = false;

        let mut batch_index = 0;
        while batch_index < ops.len() {
            let packets = split_into_packets(
                &ops[batch_index..],
                batch_index,
                dap_index,
                self.packet_size,
                jtag,
            );
            for mut packet in packets {
                let packed = packet.transfers.len();
                if let Some((blame, last)) = status_check
                    && batch_index + packed > last
                {
                    checked = packet.check_status(blame, self.packet_size);
                }
                results = self.flush_pending_transfers(&packet, results)?;
                batch_index += packed;
            }

            // Whatever ended the run is handled on its own, and cannot share a packet.
            let Some((id, op)) = ops.get(batch_index) else {
                break;
            };

            match op {
                SwdOp::Transfer { data, .. } if jtag && is_abort(op) => {
                    if let Err(error) = self.write_abort_command(*data) {
                        return Err(BatchExecutionError::new_from_debug_probe_at(
                            error,
                            results,
                            batch_index,
                        ));
                    }
                }
                SwdOp::Transfer { .. } => {
                    let run = run_length(&ops, batch_index);
                    results = self.run_transfer_block(
                        &ops[batch_index..batch_index + run],
                        batch_index,
                        results,
                    )?;
                    batch_index += run;
                    continue;
                }
                SwdOp::Sequence(bits) => {
                    if let Err(error) = self.run_swj_sequence_op(bits) {
                        return Err(BatchExecutionError::new_from_debug_probe_at(
                            error,
                            results,
                            batch_index,
                        ));
                    }
                }
                SwdOp::Idle { cycles } => {
                    if let Err(error) = self.run_swj_idle(*cycles) {
                        return Err(BatchExecutionError::new_from_debug_probe_at(
                            error,
                            results,
                            batch_index,
                        ));
                    }
                }
                SwdOp::Pins { out, select, wait } => {
                    match self.run_swj_pins_op(*out, *select, *wait) {
                        Ok(levels) => {
                            if id.should_capture() {
                                results.push(id, CommandResult::U8(levels.0));
                            }
                        }
                        Err(error) => {
                            return Err(BatchExecutionError::new_from_debug_probe_at(
                                error,
                                results,
                                batch_index,
                            ));
                        }
                    }
                }
            }

            batch_index += 1;
        }

        if let Some((blame, _)) = status_check
            && !checked
        {
            // The last transfer went as a block, or its packet had no room left.
            let mut packet = PendingBatch::new(dap_index);
            packet.check_status(blame, self.packet_size);
            results = self.flush_pending_transfers(&packet, results)?;
        }

        Ok(results)
    }

    fn handles_wait(&self) -> bool {
        cmsis_handles_wait(self.transfer_wait_retry)
    }

    fn handles_ap_pipeline(&self) -> bool {
        true
    }

    fn transfers_over_jtag(&self) -> bool {
        // `DAP_Transfer` works over JTAG once the firmware knows the scan chain.
        self.native_jtag_transfers()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe::{
        cmsisdap::commands::{
            CmsisDapDevice, Request,
            fake::{FakeProbe, Reply, standard_reply},
            transfer::LastTransferResponse,
        },
        jtag::TapState,
        swd::{Direction, Port, SwdOp},
    };
    use std::sync::{
        Arc, Mutex,
        atomic::{AtomicU32, Ordering},
    };

    fn block_response(count: u16, ack: Ack) -> TransferBlockResponse {
        TransferBlockResponse {
            transfer_count: count,
            transfer_response: LastTransferResponse {
                ack,
                protocol_error: false,
                _value_mismatch: false,
            },
            transfer_data: Vec::new(),
        }
    }

    #[test]
    fn a_late_block_reply_does_not_reach_the_next_command() {
        use crate::probe::cmsisdap::commands::{
            CmsisDapDevice, CommandId,
            fake::{FakeProbe, Reply, standard_reply},
            general::info::PacketCountCommand,
        };

        let mut blocks = 0;
        let probe = FakeProbe::shared(64, move |command| {
            if command[0] != CommandId::TransferBlock as u8 {
                return standard_reply(command, 64, 4);
            }
            let count = u16::from_le_bytes([command[2], command[3]]);
            let mut reply = vec![command[0], command[2], command[3], Ack::Ok as u8];
            for word in 0..u32::from(count) {
                reply.extend((blocks * 100 + word).to_le_bytes());
            }
            blocks += 1;
            // The probe is slow on the second block, so its reply and the third come late.
            if blocks == 2 {
                Reply::Held(reply)
            } else {
                Reply::Now(reply)
            }
        });
        let mut dap = CmsisDap::new_from_device(CmsisDapDevice::Fake(probe.clone())).unwrap();

        let mut batch = SwdBatch::new();
        let reads: Vec<_> = (0..30).map(|_| batch.read(Port::Ap, 0xC)).collect();
        let ops: Vec<_> = batch
            .iter()
            .map(|(id, op)| (id.clone(), op.clone()))
            .collect();

        let error = dap.run_transfer_block(&ops, 0, Results::new()).unwrap_err();
        let words_per_packet = dap.max_words_per_block_packet();
        assert_eq!(error.fault_operation, words_per_packet);
        let mut results = error.results;
        for (word, read) in reads.into_iter().take(words_per_packet).enumerate() {
            assert_eq!(results.take(read).ok(), Some(word as u32));
        }

        // One resynchronisation, without a wait for the third reply.
        let aborts = probe.lock().unwrap().aborts();
        assert_eq!(aborts, 1);

        let packet_count = commands::send_command(&mut dap.device, &PacketCountCommand {});
        assert_eq!(packet_count.unwrap(), 4);
    }

    #[test]
    fn a_complete_block_reply_is_a_success() {
        let response = block_response(4, Ack::Ok);
        assert!(CmsisDap::classify_block_response(&response, 4, 100).is_ok());
    }

    #[test]
    fn a_fault_names_the_transfer_that_failed() {
        // Two of the four ran, so the second is the one that faulted.
        let response = block_response(2, Ack::Fault);
        let (error, fault_operation) =
            CmsisDap::classify_block_response(&response, 4, 100).unwrap_err();

        assert!(matches!(
            error,
            BatchError::Specific(DebugProbeError::SwdTransfer(
                SwdTransferError::FaultResponse
            ))
        ));
        assert_eq!(fault_operation, 101);
    }

    #[test]
    fn a_short_count_with_no_error_is_reported_against_the_probe() {
        let response = block_response(3, Ack::Ok);
        let (error, fault_operation) =
            CmsisDap::classify_block_response(&response, 4, 100).unwrap_err();

        assert!(matches!(
            error,
            BatchError::Probe(DebugProbeError::Other(_))
        ));
        assert_eq!(fault_operation, 102);
    }

    #[test]
    fn a_count_past_the_chunk_stays_inside_it() {
        // A probe claiming more transfers than it was given must not point outside the chunk.
        let response = block_response(9, Ack::Fault);
        let (_, fault_operation) =
            CmsisDap::classify_block_response(&response, 4, 100).unwrap_err();

        assert_eq!(fault_operation, 103);
    }

    /// How many `rw` transfers one packet holds.
    fn transfers_that_fit(rw: RW, packet_size: u16) -> usize {
        let mut request = TransferRequest::empty();
        while request.has_room_for(rw, packet_size) {
            match rw {
                RW::R => request.add_read(RegisterAddress::ApRegister(0)),
                RW::W => request.add_write(RegisterAddress::ApRegister(0), 0),
            }
        }
        request.len()
    }

    #[test]
    fn a_packet_holds_more_reads_than_writes() {
        // A write spends five bytes of the command; a read spends one there and four in the reply.
        assert_eq!(transfers_that_fit(RW::W, 64), 12);
        assert_eq!(transfers_that_fit(RW::R, 64), 15);
        assert_eq!(transfers_that_fit(RW::W, 1024), 204);
        assert_eq!(transfers_that_fit(RW::R, 1024), 255);
    }

    #[test]
    fn a_packet_stops_at_the_transfer_count_field() {
        // The largest packets the count field can describe in full: 1282 bytes of writes, 1026 of
        // reads. Past those the packet has room the byte cannot express.
        assert_eq!(transfers_that_fit(RW::W, 1282), 255);
        assert_eq!(transfers_that_fit(RW::W, 1283), 255);
        assert_eq!(transfers_that_fit(RW::R, 1026), 255);
        assert_eq!(transfers_that_fit(RW::R, 1027), 255);
        assert_eq!(transfers_that_fit(RW::R, u16::MAX), 255);
    }

    #[test]
    fn a_scattered_run_fills_a_packet_up_to_the_count_field() {
        // An address write then a data read, per address. Charged the write price throughout, a
        // 1024-byte packet took 204 transfers, 102 addresses. Priced per direction the count field
        // binds first: 255 transfers, 127 addresses read in full and one whose data read falls
        // into the next packet.
        let mut ops = Vec::new();
        for _ in 0..200 {
            ops.push(transfer(Port::Ap, 0b0100, Direction::Write, 0x2000_0000));
            ops.push(transfer(Port::Ap, 0b1100, Direction::Read, 0));
        }

        let packets: Vec<_> = split_into_packets(&ops, 0, 0, 1024, false).collect();
        assert_eq!(packets[0].transfers.len(), 255);
    }

    #[test]
    fn batch_splits_at_packet_limit_without_read_flush() {
        let packet_size = 64u16;
        let max_per_packet = transfers_that_fit(RW::R, packet_size);
        let mut ops = Vec::new();
        for index in 0..(max_per_packet + 2) {
            // Alternating addresses keep the run below MIN_BLOCK_TRANSFERS, so
            // every transfer stays in DAP_Transfer.
            let addr = if index % 2 == 0 { 0b0100 } else { 0b1000 };
            ops.push(transfer(Port::Dp, addr, Direction::Read, 0));
        }

        let packets: Vec<_> = split_into_packets(&ops, 0, 0, packet_size, false).collect();
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[0].transfers.len(), max_per_packet);
        assert_eq!(packets[1].transfers.len(), 2);
    }

    /// Over JTAG, ABORT is not a register `DAP_Transfer` can reach, so it has to leave the
    /// packet to be sent with `DAP_WriteABORT`. Over SWD it is an ordinary DP write.
    #[test]
    fn abort_leaves_the_packet_only_when_asked() {
        let ops = vec![
            transfer(Port::Dp, 0b1000, Direction::Write, 0),
            transfer(Port::Dp, 0b0000, Direction::Write, 0x1e),
            transfer(Port::Dp, 0b0100, Direction::Read, 0),
        ];

        let alone: Vec<_> = split_into_packets(&ops, 0, 0, 64, true).collect();
        assert_eq!(alone.len(), 1);
        assert_eq!(alone[0].transfers.len(), 1);

        let together: Vec<_> = split_into_packets(&ops, 0, 0, 64, false).collect();
        assert_eq!(together.len(), 1);
        assert_eq!(together[0].transfers.len(), 3);
    }

    #[test]
    fn handles_wait_follows_transfer_configure() {
        assert!(!cmsis_handles_wait(0));
        assert!(cmsis_handles_wait(1));
        assert!(cmsis_handles_wait(0xffff));
    }

    fn transfer(port: Port, addr: u8, direction: Direction, data: u32) -> (HandleId, SwdOp) {
        (
            HandleId::new(),
            SwdOp::Transfer {
                port,
                addr,
                direction,
                data,
            },
        )
    }

    /// One item for [`PendingBatch::push_all`].
    fn item(
        id: &HandleId,
        addr: u8,
        data: u32,
        batch_index: usize,
    ) -> (HandleId, RegisterAddress, u32, usize) {
        (
            id.clone(),
            RegisterAddress::ApRegister(addr),
            data,
            batch_index,
        )
    }

    #[test]
    fn an_empty_packet_takes_one_transfer_however_small() {
        let mut packet = PendingBatch::new(0);
        let (id, _) = transfer(Port::Ap, 0b0100, Direction::Write, 0);

        // An empty packet takes its first transfer whatever it costs, or splitting a batch would
        // not make progress.
        assert!(packet.push_all(Direction::Write, 8, [item(&id, 0b0100, 0x2000_0000, 0)]));

        assert!(!packet.push_all(Direction::Write, 8, [item(&id, 0b0100, 0, 1)]));
        assert!(packet.push_all(Direction::Read, 64, [item(&id, 0b1100, 0, 2)]));
    }

    /// An address write followed by `reads` data reads, the shape of a block read.
    fn address_then_reads(reads: usize) -> Vec<(HandleId, SwdOp)> {
        let mut ops = vec![transfer(Port::Ap, 0b0100, Direction::Write, 0x2000_0000)];
        ops.extend((0..reads).map(|_| transfer(Port::Ap, 0b1100, Direction::Read, 0)));
        ops
    }

    #[test]
    fn a_repeat_that_fits_travels_with_the_address_that_set_it_up() {
        let packets: Vec<_> = split_into_packets(&address_then_reads(2), 0, 0, 64, false).collect();

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].transfers.len(), 3);
    }

    #[test]
    fn a_repeat_rides_along_until_the_reply_is_full() {
        // Three header bytes and four per read leave room for fifteen in a 64-byte reply, and the
        // address write costs the reply nothing.
        let fits: Vec<_> = split_into_packets(&address_then_reads(15), 0, 0, 64, false).collect();
        assert_eq!(fits.len(), 1);
        assert_eq!(fits[0].transfers.len(), 16);

        let one_more: Vec<_> =
            split_into_packets(&address_then_reads(16), 0, 0, 64, false).collect();
        assert_eq!(one_more.len(), 1);
        assert_eq!(one_more[0].transfers.len(), 1);
    }

    #[test]
    fn a_repeat_too_long_for_the_packet_is_left_to_the_block_command() {
        let packets: Vec<_> =
            split_into_packets(&address_then_reads(40), 0, 0, 64, false).collect();

        // Only the address write is packed. `run_batch` sends the reads as `DAP_TransferBlock`,
        // which is pipelined.
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].transfers.len(), 1);
    }

    #[test]
    fn a_run_that_does_not_fit_leaves_the_packet_as_it_was() {
        let mut packet = PendingBatch::new(0);
        let (id, _) = transfer(Port::Ap, 0b0100, Direction::Write, 0);

        assert!(packet.push_all(Direction::Write, 64, [item(&id, 0b0100, 0x2000_0000, 0)]));
        let (request, response) = packet.request.packet_lengths();

        // Long enough to run out of room part way, which is what the rewind is for.
        let run = (0..40).map(|i| item(&id, 0b1100, 0, i + 1));
        assert!(!packet.push_all(Direction::Read, 64, run));

        assert_eq!(packet.transfers.len(), 1);
        assert_eq!(packet.request.packet_lengths(), (request, response));
    }

    #[test]
    fn run_length_counts_the_same_access_only() {
        let ops = vec![
            transfer(Port::Ap, 0b1100, Direction::Read, 0),
            transfer(Port::Ap, 0b1100, Direction::Read, 0),
            transfer(Port::Ap, 0b1100, Direction::Read, 0),
            transfer(Port::Ap, 0b1000, Direction::Read, 0),
        ];

        assert_eq!(run_length(&ops, 0), 3);
        assert_eq!(run_length(&ops, 3), 1);
    }

    #[test]
    fn run_length_stops_at_a_write_and_at_an_idle() {
        let ops = vec![
            transfer(Port::Ap, 0b1100, Direction::Read, 0),
            transfer(Port::Ap, 0b1100, Direction::Write, 0),
        ];
        assert_eq!(run_length(&ops, 0), 1);

        let ops = vec![
            transfer(Port::Ap, 0b1100, Direction::Read, 0),
            (HandleId::new(), SwdOp::Idle { cycles: 8 }),
            transfer(Port::Ap, 0b1100, Direction::Read, 0),
        ];
        assert_eq!(run_length(&ops, 0), 1);
    }

    #[test]
    fn a_block_packet_holds_more_written_words_than_a_transfer_packet() {
        // A block sends the access once for the whole run, so its command has the room the
        // per-transfer path spends on a request byte per word.
        for packet_size in [64u16, 512, 1024] {
            assert!(block_words_per_packet(packet_size) > transfers_that_fit(RW::W, packet_size));
        }
    }

    #[test]
    fn block_read_request_encodes_the_access_once() {
        let mut request =
            TransferBlockRequest::read_request(RegisterAddress::ApRegister(0b1100), 3);
        request.dap_index = 1;

        let mut buffer = [0u8; 16];
        let size = request.to_bytes(&mut buffer).unwrap();

        assert_eq!(size, 4);
        assert_eq!(buffer[0], 1);
        assert_eq!(u16::from_le_bytes([buffer[1], buffer[2]]), 3);
        // AP, read, A2 and A3 set.
        assert_eq!(buffer[3], 0b1111);
    }

    #[test]
    fn block_write_request_appends_the_data_words() {
        let request = TransferBlockRequest::write_request(
            RegisterAddress::ApRegister(0b1100),
            vec![0x1111_2222, 0x3333_4444],
        );

        let mut buffer = [0u8; 16];
        let size = request.to_bytes(&mut buffer).unwrap();

        assert_eq!(size, 4 + 2 * 4);
        assert_eq!(u16::from_le_bytes([buffer[1], buffer[2]]), 2);
        // AP, write, A2 and A3 set.
        assert_eq!(buffer[3], 0b1101);
        assert_eq!(
            u32::from_le_bytes(buffer[4..8].try_into().unwrap()),
            0x1111_2222
        );
        assert_eq!(
            u32::from_le_bytes(buffer[8..12].try_into().unwrap()),
            0x3333_4444
        );
    }

    fn pending_transfers(n: usize) -> Vec<PendingTransfer> {
        (0..n)
            .map(|i| PendingTransfer {
                id: HandleId::new(),
                direction: Direction::Read,
                batch_index: 100 + i,
            })
            .collect()
    }

    fn transfer_response(count: usize, ack: Ack) -> TransferResponse {
        TransferResponse {
            last_transfer_response: LastTransferResponse {
                ack,
                protocol_error: false,
                _value_mismatch: false,
            },
            transfers: (0..count)
                .map(
                    |_| crate::probe::cmsisdap::commands::transfer::InnerTransferResponse {
                        td_timestamp: None,
                        data: Some(0),
                    },
                )
                .collect(),
        }
    }

    #[test]
    fn a_faulting_transfer_is_reported_as_a_fault_not_as_a_short_count() {
        // The probe stops early *because* of the FAULT, so the short count must not mask it.
        // Reporting a bare probe error here skips sticky-error recovery and leaves
        // CTRL/STAT.STICKYERR set, after which every later AP access faults (IHI0031G B4.2.4).
        let transfers = pending_transfers(4);
        let response = transfer_response(2, Ack::Fault);

        let (error, fault_operation) =
            CmsisDap::classify_transfer_response(&response, &transfers, None).unwrap_err();

        assert!(
            matches!(
                error,
                BatchError::Specific(DebugProbeError::SwdTransfer(
                    SwdTransferError::FaultResponse
                ))
            ),
            "got {error:?}"
        );
        assert_eq!(fault_operation, 101);
    }

    #[test]
    fn a_waiting_transfer_is_reported_as_a_wait_not_as_a_short_count() {
        let transfers = pending_transfers(4);
        let response = transfer_response(1, Ack::Wait);

        let (error, _) =
            CmsisDap::classify_transfer_response(&response, &transfers, None).unwrap_err();

        assert!(
            matches!(
                error,
                BatchError::Specific(DebugProbeError::SwdTransfer(SwdTransferError::WaitResponse))
            ),
            "got {error:?}"
        );
    }

    #[test]
    fn a_short_transfer_count_with_an_ok_ack_is_reported_against_the_probe() {
        let transfers = pending_transfers(4);
        let response = transfer_response(3, Ack::Ok);

        let (error, fault_operation) =
            CmsisDap::classify_transfer_response(&response, &transfers, None).unwrap_err();

        assert!(matches!(
            error,
            BatchError::Probe(DebugProbeError::Other(_))
        ));
        assert_eq!(fault_operation, 102);
    }

    #[test]
    fn a_complete_transfer_reply_is_a_success() {
        let transfers = pending_transfers(4);
        let response = transfer_response(4, Ack::Ok);

        assert!(CmsisDap::classify_transfer_response(&response, &transfers, None).is_ok());
    }

    const STICKYERR: u32 = 1 << 5;
    /// The request byte of a `DAP_Transfer` read of CTRL/STAT: DP, read, A[3:2] = 0b01.
    const READ_CTRL_STAT: u8 = 0b0110;
    /// The request byte of a `DAP_Transfer` write of AP register 0x4.
    const WRITE_AP_4: u8 = 0b0101;

    /// A CMSIS-DAP probe in JTAG mode with its scan chain configured, whose DP answers reads of
    /// CTRL/STAT with `ctrl` and every other read with the transfer's index in its command.
    /// `DAP_WriteABORT` clears STICKYERR in `ctrl`.
    fn jtag_dap(ctrl: &Arc<AtomicU32>) -> (CmsisDap, Arc<Mutex<FakeProbe>>) {
        let probe = FakeProbe::shared(64, {
            let ctrl = ctrl.clone();
            move |command| match command {
                [0x05, _, count, requests @ ..] => {
                    let mut reply = vec![0x05, *count, Ack::Ok as u8];
                    let mut offset = 0;
                    for index in 0..*count {
                        let request = requests[offset];
                        if request & 0b10 == 0 {
                            offset += 5;
                            continue;
                        }
                        let word = if request & 0b1101 == READ_CTRL_STAT & 0b1101 {
                            ctrl.load(Ordering::Relaxed)
                        } else {
                            u32::from(index)
                        };
                        reply.extend(word.to_le_bytes());
                        offset += 1;
                    }
                    Reply::Now(reply)
                }
                [0x06, _, lo, hi, request, ..] => {
                    let mut reply = vec![0x06, *lo, *hi, Ack::Ok as u8];
                    if request & 0b10 != 0 {
                        for index in 0..u32::from(u16::from_le_bytes([*lo, *hi])) {
                            reply.extend(index.to_le_bytes());
                        }
                    }
                    Reply::Now(reply)
                }
                [0x08, ..] => {
                    ctrl.fetch_and(!STICKYERR, Ordering::Relaxed);
                    Reply::Now(vec![0x08, 0x00])
                }
                _ => standard_reply(command, 64, 4),
            }
        });
        let mut dap = CmsisDap::new_from_device(CmsisDapDevice::Fake(probe.clone())).unwrap();
        dap.protocol = Some(WireProtocol::Jtag);
        dap.connected = true;
        dap.jtag_configured = true;
        dap.jtag_state.tap_state = TapState::RunTestIdle;
        probe.lock().unwrap().sent.clear();
        (dap, probe)
    }

    /// The `DAP_Transfer`, `DAP_TransferBlock` and `DAP_WriteABORT` commands the probe was sent.
    fn dap_commands(probe: &Arc<Mutex<FakeProbe>>) -> Vec<Vec<u8>> {
        probe
            .lock()
            .unwrap()
            .sent
            .iter()
            .filter(|command| matches!(command[0], 0x05 | 0x06 | 0x08))
            .cloned()
            .collect()
    }

    /// Over JTAG the probe acknowledges a refused AP transfer as OK, so the batch reads CTRL/STAT
    /// in its last packet, and a sticky error there fails the whole batch and is cleared.
    #[test]
    fn over_jtag_a_sticky_error_fails_the_batch_and_is_cleared() {
        let ctrl = Arc::new(AtomicU32::new(STICKYERR));
        let (mut dap, probe) = jtag_dap(&ctrl);
        let mut batch = SwdBatch::new();
        batch.write(Port::Ap, 0b0100, 0x2000_0000);
        let read = batch.read(Port::Ap, 0b1100);

        let error = dap.run_batch(&batch).unwrap_err();

        assert!(
            matches!(
                error.error,
                BatchError::Specific(DebugProbeError::SwdTransfer(
                    SwdTransferError::FaultResponse
                ))
            ),
            "got {:?}",
            error.error
        );
        assert_eq!(error.fault_operation, 0);
        let mut results = error.results;
        assert!(results.take(read).is_err());

        let commands = dap_commands(&probe);
        assert_eq!(commands.len(), 2, "{commands:x?}");
        // The write, the read and the CTRL/STAT read travel in one packet.
        assert_eq!(commands[0][2], 3);
        assert_eq!(commands[0].last(), Some(&READ_CTRL_STAT));
        // STKERRCLR goes through DAP_WriteABORT, which reaches ABORT over JTAG.
        assert_eq!(commands[1], vec![0x08, 0x00, 0x04, 0x00, 0x00, 0x00]);
        assert_eq!(ctrl.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn over_jtag_a_clear_ctrl_stat_leaves_the_results_alone() {
        let (mut dap, probe) = jtag_dap(&Arc::new(AtomicU32::new(0)));
        let mut batch = SwdBatch::new();
        batch.write(Port::Ap, 0b0100, 0x2000_0000);
        let read = batch.read(Port::Ap, 0b1100);

        let mut results = dap.run_batch(&batch).unwrap();

        // The read was the second transfer of its command.
        assert_eq!(results.take(read).unwrap(), 1);
        let commands = dap_commands(&probe);
        assert_eq!(commands.len(), 1, "{commands:x?}");
        assert_eq!(commands[0][2], 3);
        assert_eq!(commands[0].last(), Some(&READ_CTRL_STAT));
    }

    #[test]
    fn over_jtag_a_full_packet_is_followed_by_the_check_in_its_own() {
        let (mut dap, probe) = jtag_dap(&Arc::new(AtomicU32::new(0)));
        let mut batch = SwdBatch::new();
        // Fifteen reads fill a 64-byte reply, so the check does not fit behind them.
        let reads: Vec<_> = (0..15).map(|_| batch.read(Port::Ap, 0b1100)).collect();

        let mut results = dap.run_batch(&batch).unwrap();

        for (word, read) in reads.into_iter().enumerate() {
            assert_eq!(results.take(read).unwrap(), word as u32);
        }
        let commands = dap_commands(&probe);
        assert_eq!(commands.len(), 2, "{commands:x?}");
        assert_eq!(commands[0][2], 15);
        assert_eq!(commands[1], vec![0x05, 0x00, 0x01, READ_CTRL_STAT]);
    }

    #[test]
    fn over_jtag_the_check_follows_a_block_transfer() {
        let (mut dap, probe) = jtag_dap(&Arc::new(AtomicU32::new(0)));
        let mut batch = SwdBatch::new();
        let reads: Vec<_> = (0..40).map(|_| batch.read(Port::Ap, 0b1100)).collect();

        let mut results = dap.run_batch(&batch).unwrap();

        for read in reads {
            assert!(results.take(read).is_ok());
        }
        let commands = dap_commands(&probe);
        let blocks = commands.iter().filter(|command| command[0] == 0x06).count();
        assert_eq!(blocks, 3, "{commands:x?}");
        assert_eq!(commands.len(), blocks + 1);
        assert_eq!(commands[blocks], vec![0x05, 0x00, 0x01, READ_CTRL_STAT]);
    }

    /// A batch without an AP transfer cannot set the flag, and after a write to ABORT the flags
    /// are clear, so neither is checked. Over SWD the probe reports FAULT itself.
    #[test]
    fn only_a_jtag_batch_with_ap_transfers_is_checked() {
        let ctrl = Arc::new(AtomicU32::new(STICKYERR));
        let (mut dap, probe) = jtag_dap(&ctrl);

        let mut batch = SwdBatch::new();
        let read = batch.read(Port::Dp, 0b0100);
        let mut results = dap.run_batch(&batch).unwrap();
        assert_eq!(results.take(read).unwrap(), STICKYERR);

        let mut batch = SwdBatch::new();
        batch.write(Port::Ap, 0b0100, 0x2000_0000);
        batch.write(Port::Dp, 0b0000, 0x1e);
        dap.run_batch(&batch).unwrap();

        dap.protocol = Some(WireProtocol::Swd);
        ctrl.store(STICKYERR, Ordering::Relaxed);
        let mut batch = SwdBatch::new();
        batch.write(Port::Ap, 0b0100, 0x2000_0000);
        let _ = batch.read(Port::Ap, 0b1100);
        dap.run_batch(&batch).unwrap();

        let commands = dap_commands(&probe);
        assert_eq!(
            commands,
            vec![
                vec![0x05, 0x00, 0x01, READ_CTRL_STAT],
                vec![0x05, 0x00, 0x01, WRITE_AP_4, 0x00, 0x00, 0x00, 0x20],
                vec![0x08, 0x00, 0x1e, 0x00, 0x00, 0x00],
                vec![0x05, 0x00, 0x02, WRITE_AP_4, 0x00, 0x00, 0x00, 0x20, 0b1111],
            ],
            "{commands:x?}"
        );
    }
}
