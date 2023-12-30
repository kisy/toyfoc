#[repr(u8)]
#[allow(dead_code)]
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum CtrlCMD {
    None = 0,
    Enable(f32) = 1,
    Target(f32) = 2,
    LoopMode(f32) = 3,
    VoltageLimit(f32) = 4,
    VoltagePower(f32) = 5,
    VelocityLimit(f32) = 6,
    TorqueLimit(f32) = 7,

    TorqueP(f32) = 11,
    TorqueI(f32) = 12,
    TorqueD(f32) = 13,
    TorqueRamp(f32) = 14,
    TorqueTF(f32) = 15,

    VelocityP(f32) = 21,
    VelocityI(f32) = 22,
    VelocityD(f32) = 23,
    VelocityRamp(f32) = 24,
    VelocityTF(f32) = 25,

    PositionP(f32) = 31,
    PositionI(f32) = 32,
    PositionD(f32) = 33,
    PositionRamp(f32) = 34,
    PositionTF(f32) = 35,
}

impl CtrlCMD {
    pub fn new(id: u8, val: f32) -> Self {
        match id {
            1 => CtrlCMD::Enable(val),
            2 => CtrlCMD::Target(val),
            3 => CtrlCMD::LoopMode(val),
            4 => CtrlCMD::VoltageLimit(val),
            5 => CtrlCMD::VoltagePower(val),
            6 => CtrlCMD::VelocityLimit(val),
            7 => CtrlCMD::TorqueLimit(val),

            11 => CtrlCMD::TorqueP(val),
            12 => CtrlCMD::TorqueI(val),
            13 => CtrlCMD::TorqueD(val),
            14 => CtrlCMD::TorqueRamp(val),
            15 => CtrlCMD::TorqueTF(val),

            21 => CtrlCMD::VelocityP(val),
            22 => CtrlCMD::VelocityI(val),
            23 => CtrlCMD::VelocityD(val),
            24 => CtrlCMD::VelocityRamp(val),
            25 => CtrlCMD::VelocityTF(val),

            31 => CtrlCMD::PositionP(val),
            32 => CtrlCMD::PositionI(val),
            33 => CtrlCMD::PositionD(val),
            34 => CtrlCMD::PositionRamp(val),
            35 => CtrlCMD::PositionTF(val),

            _ => CtrlCMD::None,
        }
    }

    pub fn value(&self) -> f32 {
        match self {
            CtrlCMD::None => 0f32,
            CtrlCMD::Enable(val) => *val,
            CtrlCMD::Target(val) => *val,
            CtrlCMD::LoopMode(val) => *val,
            CtrlCMD::VoltageLimit(val) => *val,
            CtrlCMD::VoltagePower(val) => *val,
            CtrlCMD::VelocityLimit(val) => *val,
            CtrlCMD::TorqueLimit(val) => *val,

            CtrlCMD::TorqueP(val) => *val,
            CtrlCMD::TorqueI(val) => *val,
            CtrlCMD::TorqueD(val) => *val,
            CtrlCMD::TorqueRamp(val) => *val,
            CtrlCMD::TorqueTF(val) => *val,

            CtrlCMD::VelocityP(val) => *val,
            CtrlCMD::VelocityI(val) => *val,
            CtrlCMD::VelocityD(val) => *val,
            CtrlCMD::VelocityRamp(val) => *val,
            CtrlCMD::VelocityTF(val) => *val,

            CtrlCMD::PositionP(val) => *val,
            CtrlCMD::PositionI(val) => *val,
            CtrlCMD::PositionD(val) => *val,
            CtrlCMD::PositionRamp(val) => *val,
            CtrlCMD::PositionTF(val) => *val,
        }
    }

    pub fn id(&self) -> u8 {
        match self {
            CtrlCMD::None => 0,
            CtrlCMD::Enable(_) => 1,
            CtrlCMD::Target(_) => 2,
            CtrlCMD::LoopMode(_) => 3,
            CtrlCMD::VoltageLimit(_) => 4,
            CtrlCMD::VoltagePower(_) => 5,
            CtrlCMD::VelocityLimit(_) => 6,
            CtrlCMD::TorqueLimit(_) => 7,

            CtrlCMD::TorqueP(_) => 11,
            CtrlCMD::TorqueI(_) => 12,
            CtrlCMD::TorqueD(_) => 13,
            CtrlCMD::TorqueRamp(_) => 14,
            CtrlCMD::TorqueTF(_) => 15,

            CtrlCMD::VelocityP(_) => 21,
            CtrlCMD::VelocityI(_) => 22,
            CtrlCMD::VelocityD(_) => 23,
            CtrlCMD::VelocityRamp(_) => 24,
            CtrlCMD::VelocityTF(_) => 25,

            CtrlCMD::PositionP(_) => 31,
            CtrlCMD::PositionI(_) => 32,
            CtrlCMD::PositionD(_) => 33,
            CtrlCMD::PositionRamp(_) => 34,
            CtrlCMD::PositionTF(_) => 35,
        }
    }
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum DataCMD {
    None = 0,

    // Voltage power, Mode, target
    ConfBase(f32, f32, f32) = 100,

    // ramp, limit, tf
    ConfVelocity(f32, f32, f32) = 101,
    ConfPosition(f32, f32, f32) = 102,
    ConfTorque(f32, f32, f32) = 103,

    // p, i, d
    ConfTorquePID(f32, f32, f32) = 104,
    ConfVelocityPID(f32, f32, f32) = 105,
    ConfPositionPID(f32, f32, f32) = 106,

    // torque limit, velocity limit
    ConfLimit(f32, f32, f32) = 107,
    // voltage offset
    ConfVoltageOffset(f32, f32, f32) = 108,

    // velocity, position, torque
    DataBase(f32, f32, f32) = 150,
    // uq, id, iq
    DataQ(f32, f32, f32) = 151,
    // iu, iv, iw
    DataCurrent(f32, f32, f32) = 152,

    // timestamp, debug f32, debug f32
    DataTime(f32, f32, f32) = 153,
}

impl DataCMD {
    pub fn new(id: u8, first: f32, second: f32, third: f32) -> Self {
        match id {
            100 => DataCMD::ConfBase(first, second, third),
            101 => DataCMD::ConfVelocity(first, second, third),
            102 => DataCMD::ConfPosition(first, second, third),
            103 => DataCMD::ConfTorque(first, second, third),

            104 => DataCMD::ConfTorquePID(first, second, third),
            105 => DataCMD::ConfVelocityPID(first, second, third),
            106 => DataCMD::ConfPositionPID(first, second, third),

            107 => DataCMD::ConfLimit(first, second, third),
            108 => DataCMD::ConfVoltageOffset(first, second, third),

            150 => DataCMD::DataBase(first, second, third),
            151 => DataCMD::DataQ(first, second, third),
            152 => DataCMD::DataCurrent(first, second, third),
            153 => DataCMD::DataTime(first, second, third),

            _ => DataCMD::None,
        }
    }

    pub fn id(&self) -> u8 {
        match self {
            DataCMD::None => 0,

            DataCMD::ConfBase(_, _, _) => 100,
            DataCMD::ConfVelocity(_, _, _) => 101,
            DataCMD::ConfPosition(_, _, _) => 102,
            DataCMD::ConfTorque(_, _, _) => 103,

            DataCMD::ConfTorquePID(_, _, _) => 104,
            DataCMD::ConfVelocityPID(_, _, _) => 105,
            DataCMD::ConfPositionPID(_, _, _) => 106,

            DataCMD::ConfLimit(_, _, _) => 107,
            DataCMD::ConfVoltageOffset(_, _, _) => 108,

            DataCMD::DataBase(_, _, _) => 150,
            DataCMD::DataQ(_, _, _) => 151,
            DataCMD::DataCurrent(_, _, _) => 152,
            DataCMD::DataTime(_, _, _) => 153,
        }
    }

    pub fn value(&self) -> (f32, f32, f32) {
        match self {
            DataCMD::None => (0f32, 0f32, 0f32),

            DataCMD::ConfBase(first, second, third) => (*first, *second, *third),
            DataCMD::ConfVelocity(first, second, third) => (*first, *second, *third),
            DataCMD::ConfPosition(first, second, third) => (*first, *second, *third),
            DataCMD::ConfTorque(first, second, third) => (*first, *second, *third),

            DataCMD::ConfTorquePID(first, second, third) => (*first, *second, *third),
            DataCMD::ConfVelocityPID(first, second, third) => (*first, *second, *third),
            DataCMD::ConfPositionPID(first, second, third) => (*first, *second, *third),

            DataCMD::ConfLimit(first, second, third) => (*first, *second, *third),
            DataCMD::ConfVoltageOffset(first, second, third) => (*first, *second, *third),

            DataCMD::DataBase(first, second, third) => (*first, *second, *third),
            DataCMD::DataQ(first, second, third) => (*first, *second, *third),
            DataCMD::DataCurrent(first, second, third) => (*first, *second, *third),
            DataCMD::DataTime(first, second, third) => (*first, *second, *third),
        }
    }
}
