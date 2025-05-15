//! Sequences for tms570 devices
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;

use crate::MemoryMappedRegister;
use crate::architecture::arm::communication_interface::DapProbe;
use crate::architecture::arm::core::armv7a_debug_regs::{
    Dbgbcr, Dbgbvr, Dbgdrcr, Dbgdscr, Dbgdtrrx, Dbgdtrtx, Dbgitr,
};
use crate::architecture::arm::core::instructions::aarch32::{build_ldc, build_mrc, build_stc};
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{ArmDebugSequence, ArmDebugSequenceError};
use crate::architecture::arm::{ArmError, dp::DpAddress};
use crate::probe::WireProtocol;

use super::icepick::Icepick;

const TMS570_TAP_INDEX: u8 = 0;

/// Marker struct indicating initialization sequencing for cc13xx_cc26xx family parts.
#[derive(Debug)]
pub struct TMS570 {
    existing_breakpoint: AtomicU32,
    breakpoint_active: AtomicBool,
}

impl TMS570 {
    /// Create the sequencer for the cc13xx_cc26xx family of parts.
    pub fn create(_name: String) -> Arc<Self> {
        Arc::new(Self {
            existing_breakpoint: AtomicU32::new(0),
            breakpoint_active: AtomicBool::new(false),
        })
    }
}

struct Armv7AHelper<'a> {
    memory: &'a mut dyn ArmMemoryInterface,
    base_address: u64,
}

impl<'a> Armv7AHelper<'a> {
    fn new(
        memory: &'a mut dyn ArmMemoryInterface,
        base_address: Option<u64>,
    ) -> Result<Self, ArmError> {
        let base_address = base_address.ok_or(ArmError::NoArmTarget)?;
        Ok(Self {
            memory,
            base_address,
        })
    }

    fn get_hw_breakpoint(&mut self, bp_unit_index: usize) -> Result<Option<u32>, ArmError> {
        let bp_value_addr = Dbgbvr::get_mmio_address_from_base(self.base_address)
            .or(Err(ArmError::AddressOutOf32BitAddressSpace))?
            + (bp_unit_index * size_of::<u32>()) as u64;
        let bp_value = self.memory.read_word_32(bp_value_addr)?;

        let bp_control_addr = Dbgbcr::get_mmio_address_from_base(self.base_address)?
            + (bp_unit_index * size_of::<u32>()) as u64;
        let bp_control = Dbgbcr(self.memory.read_word_32(bp_control_addr)?);

        Ok(if bp_control.e() { Some(bp_value) } else { None })
    }

    fn set_hw_breakpoint(&mut self, bp_unit_index: usize, addr: u32) -> Result<(), ArmError> {
        let bp_value_addr = Dbgbvr::get_mmio_address_from_base(self.base_address)?
            + (bp_unit_index * size_of::<u32>()) as u64;
        let bp_control_addr = Dbgbcr::get_mmio_address_from_base(self.base_address)?
            + (bp_unit_index * size_of::<u32>()) as u64;
        let mut bp_control = Dbgbcr(0);

        // Breakpoint type - address match
        bp_control.set_bt(0b0000);
        // Match on all modes
        bp_control.set_hmc(true);
        bp_control.set_pmc(0b11);
        // Match on all bytes
        bp_control.set_bas(0b1111);
        // Enable
        bp_control.set_e(true);

        self.memory.write_word_32(bp_value_addr, addr)?;
        self.memory
            .write_word_32(bp_control_addr, bp_control.into())?;
        Ok(())
    }

    fn clear_hw_breakpoint(&mut self, bp_unit_index: usize) -> Result<(), ArmError> {
        let bp_value_addr = Dbgbvr::get_mmio_address_from_base(self.base_address)
            .or(Err(ArmError::AddressOutOf32BitAddressSpace))?
            + (bp_unit_index * size_of::<u32>()) as u64;
        let bp_control_addr = Dbgbcr::get_mmio_address_from_base(self.base_address)
            .or(Err(ArmError::AddressOutOf32BitAddressSpace))?
            + (bp_unit_index * size_of::<u32>()) as u64;

        self.memory.write_word_32(bp_value_addr, 0)?;
        self.memory.write_word_32(bp_control_addr, 0)?;

        Ok(())
    }

    fn wait_for_core_halted(&mut self) -> Result<(), ArmError> {
        let address = Dbgdscr::get_mmio_address_from_base(self.base_address)?;
        while !Dbgdscr(self.memory.read_word_32(address)?).halted() {}
        Ok(())
    }

    fn wait_for_core_running(&mut self) -> Result<(), ArmError> {
        let address = Dbgdscr::get_mmio_address_from_base(self.base_address)?;
        while !Dbgdscr(self.memory.read_word_32(address)?).restarted() {}
        Ok(())
    }

    fn halt(&mut self) -> Result<(), ArmError> {
        let address = Dbgdrcr::get_mmio_address_from_base(self.base_address)
            .or(Err(ArmError::OutOfBounds))?;
        let mut value = Dbgdrcr(0);
        value.set_hrq(true);
        self.memory.write_word_32(address, value.into())?;
        self.wait_for_core_halted()?;
        Ok(())
    }

    fn resume(&mut self) -> Result<(), ArmError> {
        let address = Dbgdrcr::get_mmio_address_from_base(self.base_address)
            .or(Err(ArmError::OutOfBounds))?;
        let mut value = Dbgdrcr(0);
        value.set_rrq(true);
        self.memory.write_word_32(address, value.into())?;
        self.wait_for_core_running()?;
        Ok(())
    }

    // fn clear_abort(&mut self) -> Result<(), ArmError> {
    //     let address = Dbgdrcr::get_mmio_address_from_base(self.base_address)?;
    //     let mut dbgdrcr = Dbgdrcr(0);
    //     dbgdrcr.set_cse(true);

    //     self.memory.write_word_32(address, dbgdrcr.into())?;
    //     Ok(())
    // }

    /// Execute an instruction
    fn execute_instruction(&mut self, instruction: u32) -> Result<Dbgdscr, ArmError> {
        // Run instruction
        let address = Dbgitr::get_mmio_address_from_base(self.base_address)?;
        self.memory.write_word_32(address, instruction)?;

        // Wait for completion
        let address = Dbgdscr::get_mmio_address_from_base(self.base_address)?;
        let mut dbgdscr = Dbgdscr(self.memory.read_word_32(address)?);

        while !dbgdscr.instrcoml_l() {
            dbgdscr = Dbgdscr(self.memory.read_word_32(address)?);
        }

        // Check if we had any aborts, if so clear them and fail
        if dbgdscr.adabort_l() || dbgdscr.sdabort_l() {
            let address = Dbgdrcr::get_mmio_address_from_base(self.base_address)?;
            let mut dbgdrcr = Dbgdrcr(0);
            dbgdrcr.set_cse(true);

            self.memory.write_word_32(address, dbgdrcr.into())?;
            return Err(ArmError::Armv7a(
                crate::architecture::arm::armv7a::Armv7aError::DataAbort,
            ));
        }

        Ok(dbgdscr)
    }

    fn execute_instruction_with_input(
        &mut self,
        instruction: u32,
        value: u32,
    ) -> Result<(), ArmError> {
        // Move value
        let address = Dbgdtrrx::get_mmio_address_from_base(self.base_address)?;
        self.memory.write_word_32(address, value)?;

        // Wait for RXfull
        let address = Dbgdscr::get_mmio_address_from_base(self.base_address)?;
        let mut dbgdscr = Dbgdscr(self.memory.read_word_32(address)?);

        while !dbgdscr.rxfull_l() {
            dbgdscr = Dbgdscr(self.memory.read_word_32(address)?);
        }

        // Run instruction
        self.execute_instruction(instruction)?;

        Ok(())
    }

    /// Execute an instruction on the CPU and return the result
    fn execute_instruction_with_result(&mut self, instruction: u32) -> Result<u32, ArmError> {
        // Run instruction
        let mut dbgdscr = self.execute_instruction(instruction)?;

        // Wait for TXfull
        while !dbgdscr.txfull_l() {
            let address = Dbgdscr::get_mmio_address_from_base(self.base_address)?;
            dbgdscr = Dbgdscr(self.memory.read_word_32(address)?);
        }

        // Read result
        let address = Dbgdtrtx::get_mmio_address_from_base(self.base_address)?;
        let result = self.memory.read_word_32(address)?;

        Ok(result)
    }

    fn set_r0(&mut self, value: u32) -> Result<(), ArmError> {
        let instruction = build_mrc(14, 0, 0, 0, 5, 0);

        self.execute_instruction_with_input(instruction, value)
    }

    fn write_word_32(&mut self, address: u32, data: u32) -> Result<(), ArmError> {
        // STC p14, c5, [r0], #4
        let instr = build_stc(14, 5, 0, 4);

        // Load r0 with the address to write to
        self.set_r0(address)?;
        self.execute_instruction_with_input(instr, data)?;
        Ok(())
    }

    fn read_word_32(&mut self, address: u32) -> Result<u32, ArmError> {
        // LDC p14, c5, [r0], #4
        let instr = build_ldc(14, 5, 0, 4);

        // Load r0 with the address to read from
        self.set_r0(address)?;

        // Read memory from [r0]
        let result = self.execute_instruction_with_result(instr)?;
        Ok(result)
    }
}

impl ArmDebugSequence for TMS570 {
    /// When a core is reset, it is always set to be caught
    fn reset_catch_set(
        &self,
        memory: &mut dyn ArmMemoryInterface,
        _core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let mut helper = Armv7AHelper::new(memory, debug_base)?;
        helper.halt()?;
        let existing = helper.get_hw_breakpoint(0)?;
        if let Some(existing) = existing {
            self.existing_breakpoint.store(existing, Ordering::Release);
        }
        self.breakpoint_active
            .store(existing.is_some(), Ordering::Relaxed);

        // Insert a breakpoint at address 0
        helper.set_hw_breakpoint(0, 0)?;
        helper.resume()?;
        helper.wait_for_core_running()?;

        // let arm_probe = interface.get_arm_probe_interface()?;
        // let probe = arm_probe.try_dap_probe_mut().ok_or(ArmError::NoArmTarget)?;
        // let mut icepick = Icepick::initialized(probe)?;
        // icepick.catch_reset(TMS570_TAP_INDEX)?;
        // icepick.bypass()?;
        Ok(())
    }

    fn reset_catch_clear(
        &self,
        memory: &mut dyn ArmMemoryInterface,
        _core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let mut helper = Armv7AHelper::new(memory, debug_base)?;
        helper.halt()?;
        helper.wait_for_core_halted()?;

        if self.breakpoint_active.swap(false, Ordering::Release) {
            helper.set_hw_breakpoint(0, self.existing_breakpoint.load(Ordering::Relaxed))?;
        } else {
            helper.clear_hw_breakpoint(0)?;
        }
        //     let arm_probe = interface.get_arm_probe_interface()?;
        //     let probe = arm_probe.try_dap_probe_mut().ok_or(ArmError::NoArmTarget)?;
        //     let mut icepick = Icepick::initialized(probe)?;
        //     // // Wait for the system to reset
        //     // std::thread::sleep(Duration::from_millis(10));

        //     icepick.release_from_reset(TMS570_TAP_INDEX)?;

        //     // // Wait for the system to reset
        //     // std::thread::sleep(Duration::from_millis(10));
        //     icepick.bypass()?;

        // TMS570 has ECC RAM. Ensure it's cleared to avoid cascading failures.
        helper.write_word_32(0xffff_ff5c, 0xau32)?;
        helper.write_word_32(0xffff_ff60, 1u32)?;
        while helper.read_word_32(0xffff_ff68)? & (1u32 << 8) == 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
        helper.write_word_32(0xffff_ff5c, 0x5u32)?;

        Ok(())
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _core_type: probe_rs_target::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let arm_probe = interface.get_arm_probe_interface()?;
        let probe = arm_probe.try_dap_probe_mut().ok_or(ArmError::NoArmTarget)?;
        let mut icepick = Icepick::initialized(probe)?;
        icepick.sysreset()?;
        icepick.bypass()?;

        Ok(())
    }

    fn debug_port_setup(
        &self,
        interface: &mut dyn DapProbe,
        _dp: DpAddress,
    ) -> Result<(), ArmError> {
        tracing::trace!("Configuring TMS570...");
        // Ensure current debug interface is in reset state.
        interface.swj_sequence(51, 0x0007_FFFF_FFFF_FFFF)?;

        match interface.active_protocol() {
            Some(WireProtocol::Jtag) => {
                let mut icepick = Icepick::new(interface)?;
                icepick.select_tap(TMS570_TAP_INDEX)?;

                // Call the configure JTAG function. We don't derive the scan chain at runtime
                // for these devices, but regardless the scan chain must be told to the debug probe
                // We avoid the live scan for the following reasons:
                // 1. Only the ICEPICK is connected at boot so we need to manually the CPU to the scan chain
                // 2. Entering test logic reset disconects the CPU again
                interface.configure_jtag(true)?;
            }
            Some(WireProtocol::Swd) => {
                return Err(ArmDebugSequenceError::SequenceSpecific(
                    "The tms570 family doesn't support SWD".into(),
                )
                .into());
            }
            _ => {
                return Err(ArmDebugSequenceError::SequenceSpecific(
                    "Cannot detect current protocol".into(),
                )
                .into());
            }
        }

        tracing::info!("TMS570 configured");

        Ok(())
    }
}
