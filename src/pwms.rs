use embedded_hal::{digital::v2::OutputPin, PwmPin};

use crate::ToyError;

pub struct PWMs<CHU, CHV, CHW, P> {
    pub duty_max: u16,
    pub u: CHU,
    pub v: CHV,
    pub w: CHW,
    pub en_pin: Option<P>,
}

pub trait PWMsTrait {
    fn set_duty(&mut self, duty_u: f32, duty_v: f32, duty_w: f32);
    fn enable(&mut self) -> nb::Result<(), ToyError>;
    fn disable(&mut self) -> nb::Result<(), ToyError>;
}

impl<CHU, CHV, CHW, P, E> PWMsTrait for PWMs<CHU, CHV, CHW, P>
where
    CHU: PwmPin<Duty = u16>,
    CHV: PwmPin<Duty = u16>,
    CHW: PwmPin<Duty = u16>,
    P: OutputPin<Error = E>,
{
    fn set_duty(&mut self, duty_u: f32, duty_v: f32, duty_w: f32) {
        self.u.set_duty((duty_u * self.duty_max as f32) as u16);
        self.v.set_duty((duty_v * self.duty_max as f32) as u16);
        self.w.set_duty((duty_w * self.duty_max as f32) as u16);
    }

    fn enable(&mut self) -> nb::Result<(), ToyError> {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().map_err(|_| ToyError::PWMEnErr)?;
        }

        Ok(())
    }

    fn disable(&mut self) -> nb::Result<(), ToyError> {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().map_err(|_| ToyError::PWMDisErr)?;
        } else {
            self.u.set_duty(0);
            self.v.set_duty(0);
            self.w.set_duty(0);
        }

        Ok(())
    }
}
