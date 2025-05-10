//! XDS110 commands

use crate::probe::common::{JtagState, RegisterState};

use super::Xds110Error;

/// Only part of commands are implemented
#[repr(u8)]
#[allow(dead_code)]
pub enum CommandId {
    /// Returns the value written
    Echo = 0x00,

    /// Connect to the probe, asserting its control signals
    Connect = 0x01,

    /// Disconnect from the probe, tristating its control signals
    Disconnect = 0x02,

    /// Get firmware and hardware version
    Version = 0x03,

    /// Set the delay for TCK, which adjusts the frequency
    SetTckDelay = 0x04,

    /// Set nTRST according to the value
    SetTrst = 0x05,

    /// Return the current value of nTRST
    GetTrst = 0x06,

    /// Cycle TCK by a given number of ticks. Does not change the other signals. Must send
    /// a minimum of 8 cycles.
    CycleTck = 0x07,

    GetJtagState = 0x08,
    GotoState = 0x09,
    FixedScan = 0x0a,
    SendScan = 0x0b,
    JtagScan = 0x0c,
    SetAmbleDelay = 0x0d,
    SetSrst = 0x0e,
    CmapiConnect = 0x0f,
    CmapiDisconnect = 0x10,
    CmapiAcquire = 0x11,
    CmapiRelease = 0x12,
    CmapiMemRead = 0x13,
    CmapiMemWrite = 0x14,
    CmapiRegRead = 0x15,
    CmapiRegWrite = 0x16,
    SwdConnect = 0x17,
    SwdDisconnect = 0x18,
    SwdEnableSwo = 0x19,
    SwdDisableSwo = 0x1a,
    SwdRegRead = 0x1b,
    SwdRegWrite = 0x1c,
    EtSetup = 0x1d,
    EtCalibrate = 0x1e,
    EtStart = 0x1f,
    EtStop = 0x20,
    EtCleanup = 0x21,
    EtDcdcGetMcuVersion = 0x22,
    EtDcdcPowerDownMcu = 0x23,
    EtDcdcSetVcc = 0x24,
    EtDcdcRestartMcu = 0x25,
    /* 0x26 is reserved */
    ProductTest = 0x27,
    ConnectEt = 0x28,
    RegisterRead = 0x29,
    RegisterWrite = 0x2a,
    CjtagConnect = 0x2b,
    CjtagDisconnect = 0x2c,
    SelectTap = 0x2d,
    DeselctTap = 0x2e,
    JtagIsolate = 0x2f,
    EtSetupRange = 0x30,
    EtSetupDig = 0x31,
    SetSupply = 0x32,
    C28xMemRead = 0x33,
    C28xMemWrite = 0x34,
    C64xMemRead = 0x35,
    C64xMemWrite = 0x36,
    C64xMemReadStat = 0x37,
    C64xMemWriteState = 0x38,
    PowerIsolate = 0x39,
    OcdDapRequest = 0x3a,
    OcdScanRequest = 0x3b,
    OcdPathMove = 0x3c,
    CmapiMemRead64 = 0x3d,
    CmapiMemWrite64 = 0x3e,
    EepromRead = 0x3f,
    EepromWrite = 0x40,
    StemSetup = 0x41,
    StemStart = 0x42,
    StemStop = 0x43,
    EtSetupDigParams = 0x44,
    JtagConnect = 0x45,
    EtHardwareInfo = 0x46,
    Armv7MemRead = 0x47,
    Armv7MemWrite = 0x48,
    SetProperty = 0x49,
    /* 0x4a - 0x4f are reserved */
    SecapBulkWrite = 0x50,
    SecapBulkRead = 0x51,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JtagTransit {
    Quickest = 1,
    ViaCapture = 2,
    ViaIdle = 3,
}

impl TryFrom<u32> for JtagTransit {
    type Error = &'static str;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            1 => JtagTransit::Quickest,
            2 => JtagTransit::ViaCapture,
            3 => JtagTransit::ViaIdle,
            _ => return Err("Unrecognized value"),
        })
    }
}

fn jtag_state_to_xds110(jtag: JtagState) -> u32 {
    match jtag {
        JtagState::Reset => 1,
        JtagState::Idle => 2,
        JtagState::Dr(register_state) => match register_state {
            RegisterState::Capture => 16,
            RegisterState::Select => 12,
            RegisterState::Shift => 3,
            RegisterState::Exit1 => 8,
            RegisterState::Pause => 5,
            RegisterState::Exit2 => 10,
            RegisterState::Update => 14,
        },
        JtagState::Ir(register_state) => match register_state {
            RegisterState::Capture => 17,
            RegisterState::Select => 13,
            RegisterState::Shift => 4,
            RegisterState::Exit1 => 9,
            RegisterState::Pause => 6,
            RegisterState::Exit2 => 11,
            RegisterState::Update => 15,
        },
    }
}

fn jtag_state_from_xds110(bytes: &[u8]) -> Result<JtagState, Xds110Error> {
    let value = u32::from_le_bytes(bytes.try_into().or(Err(Xds110Error::InvalidPayload))?);
    Ok(match value {
        1 => JtagState::Reset,
        2 => JtagState::Idle,
        16 => JtagState::Dr(RegisterState::Capture),
        12 => JtagState::Dr(RegisterState::Select),
        3 => JtagState::Dr(RegisterState::Shift),
        8 => JtagState::Dr(RegisterState::Exit1),
        5 => JtagState::Dr(RegisterState::Pause),
        10 => JtagState::Dr(RegisterState::Exit2),
        14 => JtagState::Dr(RegisterState::Update),
        17 => JtagState::Ir(RegisterState::Capture),
        13 => JtagState::Ir(RegisterState::Select),
        4 => JtagState::Ir(RegisterState::Shift),
        9 => JtagState::Ir(RegisterState::Exit1),
        6 => JtagState::Ir(RegisterState::Pause),
        11 => JtagState::Ir(RegisterState::Exit2),
        15 => JtagState::Ir(RegisterState::Update),
        _ => return Err(Xds110Error::InvalidPayload),
    })
}

pub(crate) trait Xds110Command {
    const COMMAND_ID: CommandId;
    type Response: Xds110CommandResponse;

    fn payload(&self) -> Vec<u8>;

    /// Convert the request to bytes, which can be sent to the probe.
    /// Returns the amount of bytes written to the buffer.
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, super::Xds110Error> {
        let payload = self.payload();
        // The `command` counts as part of the payload length
        let payload_len = TryInto::<u16>::try_into(payload.len() + 1)
            .or(Err(super::Xds110Error::InvalidPayload))?;

        buffer[0] = b'*';
        buffer[1..=2].copy_from_slice(&payload_len.to_le_bytes());
        buffer[3] = Self::COMMAND_ID as u8;
        buffer[4..payload.len() + 4].copy_from_slice(&payload);
        Ok(payload.len() + 4)
    }

    /// Parse the response to this request from received bytes.
    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, super::Xds110Error> {
        Self::Response::from_payload(buffer)
    }
}

pub(crate) trait Xds110CommandResponse {
    /// parse from the PAYLOAD part only
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error>
    where
        Self: Sized;
}

impl Xds110CommandResponse for () {
    fn from_payload(_bytes: &[u8]) -> Result<Self, Xds110Error> {
        Ok(())
    }
}
impl Xds110CommandResponse for u8 {
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error> {
        if bytes.len() != 1 {
            Err(Xds110Error::InvalidPayload)
        } else {
            Ok(bytes[0])
        }
    }
}

/// Tell the probe we're connecting to it
#[derive(Debug)]
pub struct Connect;

impl Xds110Command for Connect {
    const COMMAND_ID: CommandId = CommandId::Connect;
    type Response = GenericResponse;

    fn payload(&self) -> Vec<u8> {
        vec![]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GenericResponse {
    pub result: i32,
}

impl Xds110CommandResponse for GenericResponse {
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error> {
        if bytes.len() < 4 {
            return Err(Xds110Error::InvalidPayload);
        }

        Ok(GenericResponse {
            result: i32::from_le_bytes(bytes[0..4].try_into().unwrap()),
        })
    }
}

/// Set the delay for the TCK signal
#[derive(Debug)]
pub struct SetTckDelay(pub u32);

impl Xds110Command for SetTckDelay {
    const COMMAND_ID: CommandId = CommandId::SetTckDelay;

    type Response = GenericResponse;

    fn payload(&self) -> Vec<u8> {
        self.0.to_le_bytes().to_vec()
    }
}

#[derive(Debug)]
pub struct GetJtagState;

impl Xds110Command for GetJtagState {
    const COMMAND_ID: CommandId = CommandId::GetJtagState;

    type Response = GetJtagStateResponse;

    fn payload(&self) -> Vec<u8> {
        vec![]
    }
}

#[derive(Debug)]
pub struct GotoJtagState {
    pub state: JtagState,
    pub transit: JtagTransit,
}

impl Xds110Command for GotoJtagState {
    const COMMAND_ID: CommandId = CommandId::GotoState;

    type Response = GenericResponse;

    fn payload(&self) -> Vec<u8> {
        let mut payload = vec![];
        payload.extend_from_slice(&jtag_state_to_xds110(self.state).to_le_bytes());
        payload.extend_from_slice(&(self.transit as u32).to_le_bytes());
        payload
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GetJtagStateResponse {
    result: u32,
    jtag_state: JtagState,
}

impl Xds110CommandResponse for GetJtagStateResponse {
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error>
    where
        Self: Sized,
    {
        if bytes.len() < 8 {
            return Err(Xds110Error::InvalidPayload);
        }
        let jtag_state = jtag_state_from_xds110(&bytes[4..8])?;
        Ok(GetJtagStateResponse {
            result: u32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            jtag_state,
        })
    }
}

/// Cycle TCLK this many times
#[derive(Debug)]
pub struct CycleTclk(pub u32);

impl Xds110Command for CycleTclk {
    const COMMAND_ID: CommandId = CommandId::CycleTck;

    type Response = GenericResponse;

    fn payload(&self) -> Vec<u8> {
        if self.0 > 0 && self.0 < 8 {
            tracing::error!(
                "Number of TCLK cycles is {} -- will be rounded up to the minimum of 8",
                self.0
            );
        }
        self.0.to_le_bytes().to_vec()
    }
}

/// Set the nTRST pin
#[derive(Debug)]
pub struct SetTrst(pub bool);

impl Xds110Command for SetTrst {
    const COMMAND_ID: CommandId = CommandId::SetTrst;

    type Response = GenericResponse;

    fn payload(&self) -> Vec<u8> {
        vec![self.0 as u8]
    }
}

/// Get the state of the nTRST pin
#[derive(Debug)]
pub struct GetTrst;

impl Xds110Command for GetTrst {
    const COMMAND_ID: CommandId = CommandId::GetTrst;

    type Response = GetTrstResponse;

    fn payload(&self) -> Vec<u8> {
        vec![]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GetTrstResponse {
    pub result: u32,
    pub trst: bool,
}

impl Xds110CommandResponse for GetTrstResponse {
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error>
    where
        Self: Sized,
    {
        if bytes.len() < 5 {
            return Err(Xds110Error::InvalidPayload);
        }

        Ok(Self {
            result: u32::from_be_bytes(bytes[0..=3].try_into().unwrap()),
            trst: bytes[4] != 0,
        })
    }
}

/// Get current probe info, version, etc
#[derive(Debug)]
pub struct GetProbeInfo;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GetProbeInfoResponse {
    pub result: [u8; 4],
    pub version: [u8; 4],
    pub hardware: u16,
}

impl Xds110CommandResponse for GetProbeInfoResponse {
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error> {
        if bytes.len() < 3 {
            return Err(Xds110Error::InvalidPayload);
        }

        Ok(GetProbeInfoResponse {
            result: bytes[0..=3].try_into().unwrap(),
            version: bytes[4..=7].try_into().unwrap(),
            hardware: u16::from_be_bytes(bytes[8..=9].try_into().unwrap()),
        })
    }
}

impl Xds110Command for GetProbeInfo {
    const COMMAND_ID: CommandId = CommandId::Version;
    type Response = GetProbeInfoResponse;

    fn payload(&self) -> Vec<u8> {
        vec![]
    }
}

pub struct JtagScan {
    /// Total number of bits to scan
    pub bits: u16,

    /// IR vs DR path
    pub path: JtagState,

    /// Start state route
    pub trans1: JtagTransit,

    /// JTAG state after scan
    pub end_state: JtagState,

    /// End state route
    pub trans2: JtagTransit,

    /// Number of preamble bits
    pub pre: u16,

    /// Number of postamble bits
    pub post: u16,

    /// Number of extra TCKs after scan
    pub delay: u16,

    /// Number of repetitions
    pub rep: u16,

    /// Data to be transmitted
    pub out_data: Vec<u8>,

    /// The number of bytes to expect back
    pub response_length: u16,
}

impl core::fmt::Debug for JtagScan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "JtagScan {{ bits: {}, path: {:?}, trans1: {:?}, end_state: {:?}, trans2: {:?}, pre: {}, post: {}, delay: {}, rep: {}, out_data: {:02x?}, response_length: {} }}",
            self.bits,
            self.path,
            self.trans1,
            self.end_state,
            self.trans2,
            self.pre,
            self.post,
            self.delay,
            self.rep,
            self.out_data,
            self.response_length
        )
    }
}

impl Xds110Command for JtagScan {
    const COMMAND_ID: CommandId = CommandId::JtagScan;

    type Response = JtagScanResponse;

    fn payload(&self) -> Vec<u8> {
        let mut payload = vec![];
        payload.extend_from_slice(&self.bits.to_le_bytes());
        payload.extend_from_slice(&(jtag_state_to_xds110(self.path) as u8).to_le_bytes());
        payload.extend_from_slice(&(self.trans1 as u8).to_le_bytes());
        payload.extend_from_slice(&(jtag_state_to_xds110(self.end_state) as u8).to_le_bytes());
        payload.extend_from_slice(&(self.trans2 as u8).to_le_bytes());
        payload.extend_from_slice(&self.pre.to_le_bytes());
        payload.extend_from_slice(&self.post.to_le_bytes());
        payload.extend_from_slice(&self.delay.to_le_bytes());
        payload.extend_from_slice(&self.rep.to_le_bytes());
        payload.extend_from_slice(&(self.out_data.len() as u16).to_le_bytes());
        // Make the input buffer be the same length
        payload.extend_from_slice(&self.response_length.to_le_bytes());
        // Stick the output data onto the output buffer
        payload.extend_from_slice(&self.out_data);
        payload
    }
}

#[derive(Clone, PartialEq, Eq)]
pub struct JtagScanResponse {
    pub result: i32,
    pub data: Vec<u8>,
}

impl core::fmt::Debug for JtagScanResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "JtagScanResponse {{ result: {}, data: {:02x?} }}",
            self.result, self.data
        )
    }
}

impl Xds110CommandResponse for JtagScanResponse {
    fn from_payload(bytes: &[u8]) -> Result<Self, Xds110Error>
    where
        Self: Sized,
    {
        if bytes.len() < 4 {
            return Err(Xds110Error::InvalidPayload);
        }
        Ok(JtagScanResponse {
            result: i32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            data: bytes[4..].to_vec(),
        })
    }
}

#[derive(Debug)]
pub struct CjtagConnect(pub u32);

impl Xds110Command for CjtagConnect {
    const COMMAND_ID: CommandId = CommandId::CjtagConnect;

    type Response = GenericResponse;

    fn payload(&self) -> Vec<u8> {
        self.0.to_le_bytes().to_vec()
    }
}
