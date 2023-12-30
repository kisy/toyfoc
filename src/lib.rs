#![no_std]
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use ctrl::{CtrlCMD, DataCMD};
use mt6701::AngleSensorTrait;
use utils::{_abs, _floor, _PI_2, _PI_3};

pub mod ctrl;
pub mod current;
pub mod none;
pub mod pid;
pub mod pwms;
pub mod utils;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToyError {
    ADCNumErr,
    ADCReadErr,
    PWMEnErr,
    PWMDisErr,
    SensorErr,
    SensorInitErr,
    SensorCurrentInitErr,
    SVPWMErr,
}

use crate::{
    current::CurrentSensorTrait,
    pid::PID,
    pwms::PWMsTrait,
    utils::{_constrain, _cos, _electrical_angle, _normalize_angle, _sin, _3PI_2, _SQRT3},
};

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum LoopMode {
    OpenVelocity = 0,
    PositionWithSensor = 1,
    VelocityWithSensor = 2,
    PositionWithoutTorque = 3,
    Torque = 4,
    Velocity = 5,
    Position = 6,
}

#[derive(Debug, Clone, Copy)]
pub struct PhaseDuty {
    pub u: f32,
    pub v: f32,
    pub w: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct PhaseVoltages {
    pub u: f32,
    pub v: f32,
    pub w: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct PhaseCurrent {
    pub u: f32,
    pub v: f32,
    pub w: f32,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct FOCStates {
    pub conf_base: DataCMD,
    pub conf_velocity: DataCMD,
    pub conf_position: DataCMD,
    pub conf_torque: DataCMD,
    pub conf_velocity_pid: DataCMD,
    pub conf_position_pid: DataCMD,
    pub conf_torque_pid: DataCMD,
    pub conf_limit: DataCMD,
    pub conf_voltage_offset: DataCMD,
    pub data_base: DataCMD,
    pub data_q: DataCMD,
    pub data_current: DataCMD,
    pub data_time: DataCMD,
}

#[derive()]
pub struct ToyFOC<PS, S, CS> {
    pub mode: LoopMode,
    pub voltage_power: f32,
    pub voltage_limit: f32,
    pub phase_resistance: f32,
    pub pwms: PS,
    pub sensor_direction: i8,
    pub pole_pairs: u8,
    pub use_svpwm: bool,
    pub sensor: Option<S>,
    pub current_sensor: Option<CS>,
    pub electrical_angle: f32,
    pub zero_electric_angle: f32,
    pub shaft_angle: f32,
    pub target: f32,
    pub uq: f32,
    pub id: f32,
    pub iq: f32,
    pub velocity: PID,
    pub torque: PID,
    pub position: PID,
    pub velocity_limit: f32,
    pub torque_limit: f32,
    pub prev_us: u64,
    pub now_us: u64,
    pub filter_us: u64,
    pub filter_prev: f32,
    pub angle_val: f32, // single turn angle
    pub phase_duty: PhaseDuty,
    pub phase_voltages: PhaseVoltages,
    pub phase_current: PhaseCurrent,
    pub debug: f32,
}

impl<PS, CS, S, E> ToyFOC<PS, S, CS>
where
    PS: PWMsTrait,
    S: AngleSensorTrait<Error = E>,
{
    pub fn new(pwms: PS) -> Self {
        let voltage_power = 4.6;
        let voltage_limit = voltage_power / 2.5;

        ToyFOC {
            mode: LoopMode::OpenVelocity,
            voltage_power,
            voltage_limit,
            phase_resistance: 0.0,
            pwms,
            sensor_direction: 1,
            pole_pairs: 7,
            use_svpwm: false,
            sensor: None,
            current_sensor: None,
            electrical_angle: 0.0,
            zero_electric_angle: 0.0,
            shaft_angle: 0.0,
            target: 0.0,
            uq: 0.0,
            id: 0.0,
            iq: 0.0,
            velocity_limit: 0.0,
            torque_limit: 0.0,
            prev_us: 0,
            now_us: 0,
            filter_us: 0,
            filter_prev: 0.0,
            angle_val: 0.0,
            phase_duty: PhaseDuty {
                u: 0.0,
                v: 0.0,
                w: 0.0,
            },
            phase_voltages: PhaseVoltages {
                u: 0.0,
                v: 0.0,
                w: 0.0,
            },
            phase_current: PhaseCurrent {
                u: 0.0,
                v: 0.0,
                w: 0.0,
            },
            velocity: PID::new(0.2, 0f32, 0f32, 1_000.0, voltage_limit, 0.01),
            torque: PID::new(0.8, 1f32, 0f32, 1_000.0, 1.0, 0.05),
            position: PID::new(0.125, 0f32, 0f32, 1_000.0, 25.0, 0.0),
            debug: 0.0,
        }
    }

    pub fn init<D>(&mut self, mut delay: D, delay_ms: u32) -> nb::Result<(), ToyError>
    where
        D: DelayMs<u32> + DelayUs<u32>,
        CS: CurrentSensorTrait,
    {
        if let Some(ref mut current_sensor) = self.current_sensor {
            current_sensor
                .init(&mut delay)
                .map_err(|_| ToyError::SensorCurrentInitErr)?;
        }

        if let Some(ref mut sensor) = self.sensor {
            sensor.init().map_err(|_| ToyError::SensorInitErr)?;
        }

        self.pwms.enable()?;

        self.set_torque(self.voltage_limit, _3PI_2);

        delay.delay_ms(delay_ms);

        self.update_sensor()?;

        self.zero_electric_angle = self.update_electrical_angle();

        self.pwms.set_duty(0.0, 0.0, 0.0);

        delay.delay_ms(1000);

        Ok(())
    }

    pub fn update_electrical_angle(&mut self) -> f32 {
        let pp = self.pole_pairs as f32;
        let dir = self.sensor_direction as f32;
        let angle_el = dir * pp * self.angle_val - self.zero_electric_angle;

        self.electrical_angle = _normalize_angle(angle_el);

        self.electrical_angle
    }

    pub fn update_sensor(&mut self) -> nb::Result<(), ToyError> {
        if let Some(ref mut sensor) = self.sensor {
            sensor
                .update(self.now_us)
                .map_err(|_| ToyError::SensorErr)?;
            self.position.val = sensor.get_position() as f32;
            self.velocity.val = sensor.get_velocity();
            self.angle_val = sensor.get_angle();
        }

        Ok(())
    }

    pub fn update_current_sensor(&mut self) -> nb::Result<(), ToyError>
    where
        CS: CurrentSensorTrait,
    {
        if let Some(ref mut current_sensor) = self.current_sensor {
            (self.id, self.iq) = current_sensor.read_current(self.electrical_angle).unwrap();
            self.phase_current = current_sensor.get_phase_current();
        }

        Ok(())
    }

    // https://github.com/peng-zhihui/Ctrl-FOC-Lite
    // 2.Firmware/SimpleFOC_version/Ctrl-FOC-Lite-fw/lib/Arduino-FOC/src/BLDCMotor.cpp
    fn cal_duty_cycles(&mut self, uq: f32, angle_el: f32) -> PhaseDuty {
        let uq = _constrain(uq, -self.voltage_limit, self.voltage_limit);

        // Inverse Parker Transform
        let u_alpha = -uq * _sin(angle_el);
        let u_beta = uq * _cos(angle_el);

        if !self.use_svpwm {
            // Inverse Clarke Transform
            let center = self.voltage_limit / 2.0;

            self.phase_voltages = PhaseVoltages {
                u: u_alpha + center,
                v: (_SQRT3 * u_beta - u_alpha) / 2.0 + center,
                w: (-u_alpha - _SQRT3 * u_beta) / 2.0 + center,
            };

            self.phase_duty = PhaseDuty {
                u: self.phase_voltages.u / self.voltage_power,
                v: self.phase_voltages.v / self.voltage_power,
                w: self.phase_voltages.w / self.voltage_power,
            };

            return self.phase_duty;
        }

        let u_out = uq / self.voltage_power;
        let angle_el = _normalize_angle(angle_el + _PI_2);

        let sector = _floor(angle_el / _PI_3) as u8 + 1;

        let t1 = _SQRT3 * _sin(sector as f32 * _PI_3 - angle_el) * u_out;
        let t2 = _SQRT3 * _sin(angle_el - (sector - 1) as f32 * _PI_3) * u_out;
        let t0 = 1.0 - t1 - t2;

        self.phase_duty = match sector {
            1 => PhaseDuty {
                u: t1 + t2 + t0 / 2.0,
                v: t2 + t0 / 2.0,
                w: t0 / 2.0,
            },

            2 => PhaseDuty {
                u: t1 + t0 / 2.0,
                v: t1 + t2 + t0 / 2.0,
                w: t0 / 2.0,
            },

            3 => PhaseDuty {
                u: t0 / 2.0,
                v: t1 + t2 + t0 / 2.0,
                w: t2 + t0 / 2.0,
            },

            4 => PhaseDuty {
                u: t0 / 2.0,
                v: t1 + t0 / 2.0,
                w: t1 + t2 + t0 / 2.0,
            },

            5 => PhaseDuty {
                u: t2 + t0 / 2.0,
                v: t0 / 2.0,
                w: t1 + t2 + t0 / 2.0,
            },

            6 => PhaseDuty {
                u: t1 + t2 + t0 / 2.0,
                v: t0 / 2.0,
                w: t1 + t0 / 2.0,
            },

            _ => PhaseDuty {
                u: 0.0,
                v: 0.0,
                w: 0.0,
            },
        };

        self.phase_voltages = PhaseVoltages {
            u: self.phase_duty.u * self.voltage_power,
            v: self.phase_duty.v * self.voltage_power,
            w: self.phase_duty.w * self.voltage_power,
        };

        self.phase_duty
    }

    pub fn set_torque(&mut self, uq: f32, angle_el: f32) {
        self.cal_duty_cycles(uq, angle_el);

        self.phase_duty.u = _constrain(self.phase_duty.u, 0.0, 1.0);
        self.phase_duty.v = _constrain(self.phase_duty.v, 0.0, 1.0);
        self.phase_duty.w = _constrain(self.phase_duty.w, 0.0, 1.0);

        // setting duty cycles
        self.pwms
            .set_duty(self.phase_duty.u, self.phase_duty.v, self.phase_duty.w);
    }

    pub fn velocity_open_loop(&mut self) -> f32 {
        let mut ts = (self.now_us - self.prev_us) as f32 * 1e-6;
        if ts <= 0.0 || ts > 0.5 {
            ts = 1e-3;
        }

        self.shaft_angle = _normalize_angle(self.shaft_angle + self.target * ts);

        let uq = self.voltage_limit;

        let electrical_angle = _electrical_angle(self.shaft_angle, self.pole_pairs as u32);
        self.set_torque(uq, electrical_angle);

        return uq;
    }

    pub fn position_with_sensor_loop(&mut self) -> f32 {
        let error = self.target - self.position.val;
        let iq = self.position.p * error;

        let uq = if self.phase_resistance > 0.0 {
            self.phase_resistance * iq
        } else {
            iq
        };

        self.set_torque(uq, self.electrical_angle);

        uq
    }

    pub fn velocity_with_sensor_loop(&mut self) -> f32 {
        let error = self.target - self.velocity.filter_val;

        let velocity_output = self.velocity.pid(error, self.now_us);

        let iq = velocity_output;

        let uq = if self.phase_resistance > 0.0 {
            self.phase_resistance * iq
        } else {
            iq
        };

        self.set_torque(uq, self.electrical_angle);

        uq
    }

    pub fn position_without_torque_loop(&mut self) -> f32 {
        let error = self.target - self.position.val;
        let position_output = self.position.pid(error, self.now_us);

        let velocity_error = position_output - self.velocity.filter_val;
        let velocity_output: f32 = self.velocity.pid(velocity_error, self.now_us);

        let torque_error = velocity_output - self.torque.filter_val;
        let iq = self.torque.pid(torque_error, self.now_us);

        let uq = if self.phase_resistance > 0.0 {
            self.phase_resistance * iq
        } else {
            iq
        };

        self.set_torque(uq, self.electrical_angle);

        uq
    }

    pub fn torque_loop(&mut self) -> f32 {
        let error = self.target - self.torque.filter_val;

        let iq = self.torque.pid(error, self.now_us);

        let uq = if self.phase_resistance > 0.0 {
            self.phase_resistance * iq
        } else {
            iq
        };

        self.set_torque(uq, self.electrical_angle);

        uq
    }

    pub fn velocity_loop(&mut self) -> f32 {
        let error = self.target - self.velocity.filter_val;

        let velocity_output = self.velocity.pid(error, self.now_us);

        let torque_error = velocity_output - self.torque.filter_val;
        let iq = self.torque.pid(torque_error, self.now_us);

        let uq = if self.phase_resistance > 0.0 {
            self.phase_resistance * iq
        } else {
            iq
        };

        self.set_torque(uq, self.electrical_angle);

        uq
    }

    pub fn position_loop(&mut self) -> f32 {
        let error = self.target - self.position.val;
        let angle_output = self.position.pid(error, self.now_us);

        let velocity_error = angle_output - self.velocity.filter_val;
        let velocity_output: f32 = self.velocity.pid(velocity_error, self.now_us);

        let torque_error = velocity_output - self.torque.filter_val;
        let iq = self.torque.pid(torque_error, self.now_us);

        let uq = if self.phase_resistance > 0.0 {
            self.phase_resistance * iq
        } else {
            iq
        };

        self.set_torque(uq, self.electrical_angle);

        uq
    }

    pub fn run(&mut self, now_us: u64, cmd: CtrlCMD) -> nb::Result<(), ToyError>
    where
        CS: CurrentSensorTrait,
    {
        self.now_us = now_us;
        self.exec_cmd(cmd);

        self.velocity.limit = self.voltage_limit;
        self.position.limit = self.velocity_limit;
        self.torque.limit = self.torque_limit;

        self.update_sensor()?;
        self.update_electrical_angle();
        self.position.filter_val = self.position.val;

        self.debug = self.angle_val;

        self.velocity.filter_val = self
            .velocity
            .low_pass_filter(self.velocity.val, self.now_us);

        self.update_current_sensor()?;

        self.torque.val = self.iq;
        self.torque.filter_val = self.torque.low_pass_filter(self.torque.val, self.now_us);

        self.uq = match self.mode {
            LoopMode::OpenVelocity => self.velocity_open_loop(),

            LoopMode::VelocityWithSensor => self.velocity_with_sensor_loop(),
            LoopMode::PositionWithSensor => self.position_with_sensor_loop(),
            LoopMode::PositionWithoutTorque => self.position_without_torque_loop(),

            LoopMode::Torque => self.torque_loop(),
            LoopMode::Velocity => self.velocity_loop(),
            LoopMode::Position => self.position_loop(),
        };

        self.prev_us = self.now_us;

        Ok(())
    }

    pub fn get_states(&self) -> FOCStates {
        let now_sec = (self.now_us / 1000) as f32 * 1e-3;

        FOCStates {
            conf_base: DataCMD::ConfBase(self.voltage_power, self.mode as u8 as f32, self.target),
            conf_velocity: DataCMD::ConfVelocity(
                self.velocity.output_ramp,
                self.velocity.limit,
                self.velocity.filter_tf,
            ),
            conf_position: DataCMD::ConfPosition(
                self.position.output_ramp,
                self.position.limit,
                self.position.filter_tf,
            ),
            conf_torque: DataCMD::ConfTorque(
                self.torque.output_ramp,
                self.torque.limit,
                self.torque.filter_tf,
            ),
            conf_velocity_pid: DataCMD::ConfVelocityPID(
                self.velocity.p,
                self.velocity.i,
                self.velocity.d,
            ),
            conf_position_pid: DataCMD::ConfPositionPID(
                self.position.p,
                self.position.i,
                self.position.d,
            ),
            conf_torque_pid: DataCMD::ConfTorquePID(self.torque.p, self.torque.i, self.torque.d),
            conf_limit: DataCMD::ConfLimit(
                self.voltage_limit,
                self.torque_limit,
                self.velocity_limit,
            ),
            conf_voltage_offset: DataCMD::ConfVoltageOffset(0.0, 0.0, 0.0),
            data_base: DataCMD::DataBase(
                self.velocity.filter_val,
                self.position.val,
                self.torque.filter_val,
            ),
            data_q: DataCMD::DataQ(self.uq, self.id, self.iq),
            data_current: DataCMD::DataCurrent(
                self.phase_current.u,
                self.phase_current.v,
                self.phase_current.w,
            ),
            data_time: DataCMD::DataTime(now_sec, self.zero_electric_angle, self.debug),
        }
    }

    fn exec_cmd(&mut self, cmd: CtrlCMD) {
        match cmd {
            CtrlCMD::None => {}
            CtrlCMD::Enable(val) => {
                if val >= 1.0 {
                    self.pwms.enable().unwrap();
                } else {
                    self.pwms.disable().unwrap();
                }
            }

            CtrlCMD::LoopMode(val) => {
                self.mode = match val as u8 {
                    0 => LoopMode::OpenVelocity,
                    1 => LoopMode::PositionWithSensor,
                    2 => LoopMode::VelocityWithSensor,
                    3 => LoopMode::PositionWithoutTorque,
                    4 => LoopMode::Torque,
                    5 => LoopMode::Velocity,
                    6 => LoopMode::Position,
                    _ => LoopMode::OpenVelocity,
                };
            }
            CtrlCMD::Target(val) => {
                self.target = val;
            }
            CtrlCMD::VoltageLimit(val) => {
                self.voltage_limit = _constrain(val, 0.0, self.voltage_power);
            }
            CtrlCMD::VoltagePower(val) => {
                self.voltage_power = val;
            }
            CtrlCMD::VelocityLimit(val) => {
                self.velocity_limit = val;
            }
            CtrlCMD::TorqueLimit(val) => {
                self.torque_limit = val;
            }

            CtrlCMD::TorqueP(val) => {
                self.torque.p = _abs(val);
            }
            CtrlCMD::TorqueI(val) => {
                self.torque.i = _abs(val);
            }
            CtrlCMD::TorqueD(val) => {
                self.torque.d = _abs(val);
            }
            CtrlCMD::TorqueRamp(val) => {
                self.torque.output_ramp = _abs(val);
            }
            CtrlCMD::TorqueTF(val) => {
                self.torque.filter_tf = _abs(val);
            }

            CtrlCMD::VelocityP(val) => {
                self.velocity.p = _abs(val);
            }
            CtrlCMD::VelocityI(val) => {
                self.velocity.i = _abs(val);
            }
            CtrlCMD::VelocityD(val) => {
                self.velocity.d = _abs(val);
            }
            CtrlCMD::VelocityRamp(val) => {
                self.velocity.output_ramp = _abs(val);
            }
            CtrlCMD::VelocityTF(val) => {
                self.velocity.filter_tf = _abs(val);
            }

            CtrlCMD::PositionP(val) => {
                self.position.p = _abs(val);
            }
            CtrlCMD::PositionI(val) => {
                self.position.i = _abs(val);
            }
            CtrlCMD::PositionD(val) => {
                self.position.d = _abs(val);
            }
            CtrlCMD::PositionRamp(val) => {
                self.position.output_ramp = _abs(val);
            }
            CtrlCMD::PositionTF(val) => {
                self.position.filter_tf = _abs(val);
            }
        };
    }
}
