use super::{GdbErrorExt, RuntimeTarget};

use gdbstub::{
    arch::Arch,
    target::ext::breakpoints::{
        Breakpoints, HwBreakpoint, HwBreakpointOps, HwWatchpointOps, SwBreakpointOps,
    },
};

impl Breakpoints for RuntimeTarget<'_> {
    fn support_sw_breakpoint(&mut self) -> Option<SwBreakpointOps<'_, Self>> {
        None
    }

    fn support_hw_breakpoint(&mut self) -> Option<HwBreakpointOps<'_, Self>> {
        Some(self)
    }

    fn support_hw_watchpoint(&mut self) -> Option<HwWatchpointOps<'_, Self>> {
        None
    }
}

impl HwBreakpoint for RuntimeTarget<'_> {
    fn add_hw_breakpoint(
        &mut self,
        addr: u64,
        _kind: <Self::Arch as Arch>::BreakpointKind,
    ) -> gdbstub::target::TargetResult<bool, Self> {
        let mut session = self.session.lock();

        for core_id in &self.cores {
            let Ok(mut core) = session.core(*core_id) else {
                continue;
            };

            core.set_hw_breakpoint(addr).expect("breakage");
        }

        Ok(true)
    }

    fn remove_hw_breakpoint(
        &mut self,
        addr: u64,
        _kind: <Self::Arch as Arch>::BreakpointKind,
    ) -> gdbstub::target::TargetResult<bool, Self> {
        let mut session = self.session.lock();

        for core_id in &self.cores {
            let Ok(mut core) = session.core(*core_id) else {
                continue;
            };

            core.clear_hw_breakpoint(addr).expect("breakage");
        }

        Ok(true)
    }
}
