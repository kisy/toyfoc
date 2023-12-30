use embedded_hal::adc::{Channel, OneShot};
use embedded_hal::blocking::delay::DelayUs;

use crate::utils::{_cos, _sin, _1_SQRT3, _2_SQRT3};
use crate::{PhaseCurrent, ToyError};

#[derive(Debug)]
pub struct InlineCurrentSensor<ADC, UP, VP, WP> {
    pub adc_handle: ADC,

    pub adc_u_pin: Option<UP>,
    pub adc_v_pin: Option<VP>,
    pub adc_w_pin: Option<WP>,

    pub iu: f32,
    pub iv: f32,
    pub iw: f32,

    pub id: f32,
    pub iq: f32,

    pub offset_adc_u: f32,
    pub offset_adc_v: f32,
    pub offset_adc_w: f32,

    pub gain_u: f32,
    pub gain_v: f32,
    pub gain_w: f32,

    pub adc_conversion_factor: f32,
}

pub trait CurrentSensorTrait {
    fn init<D>(&mut self, delay: &mut D) -> nb::Result<(), ToyError>
    where
        D: DelayUs<u32>;
    fn read_adc(&mut self) -> nb::Result<(Option<u32>, Option<u32>, Option<u32>), ToyError>;
    fn read_current(&mut self, angle_el: f32) -> nb::Result<(f32, f32), ToyError>;
    fn cal_iq_id(&mut self, angle_el: f32);
    fn get_phase_current(&mut self) -> PhaseCurrent;
    fn get_current(&mut self) -> (f32, f32);
    fn get_offset(&mut self) -> (f32, f32, f32);
}

impl<ADC, UP, VP, WP> CurrentSensorTrait for InlineCurrentSensor<ADC, UP, VP, WP>
where
    UP: Channel<ADC>,
    VP: Channel<ADC>,
    WP: Channel<ADC>,
    ADC: OneShot<ADC, u32, UP> + OneShot<ADC, u32, VP> + OneShot<ADC, u32, WP>,
{
    fn init<D>(&mut self, delay: &mut D) -> nb::Result<(), ToyError>
    where
        D: DelayUs<u32>,
    {
        let mut adc_pin_num = 3;

        if let None = self.adc_u_pin {
            adc_pin_num -= 1
        }

        if let None = self.adc_v_pin {
            adc_pin_num -= 1
        }

        if let None = self.adc_w_pin {
            adc_pin_num -= 1
        }

        if adc_pin_num < 2 {
            return Err(nb::Error::Other(ToyError::ADCNumErr));
        }

        let mut sum_u = 0;
        let mut sum_v = 0;
        let mut sum_w = 0;

        for _ in 0..1000 {
            let (u, v, w) = self.read_adc().map_err(|_| ToyError::ADCReadErr)?;

            if let Some(val) = u {
                sum_u += val;
            }

            if let Some(val) = v {
                sum_v += val;
            }

            if let Some(val) = w {
                sum_w += val;
            }

            delay.delay_us(5);
        }

        self.offset_adc_u = sum_u as f32 / 1000.0;
        self.offset_adc_v = sum_v as f32 / 1000.0;
        self.offset_adc_w = sum_w as f32 / 1000.0;

        Ok(())
    }

    fn read_adc(&mut self) -> nb::Result<(Option<u32>, Option<u32>, Option<u32>), ToyError> {
        let adc_u: Option<u32> = match &mut self.adc_u_pin {
            Some(adc_pin) => {
                let val = self
                    .adc_handle
                    .read(adc_pin)
                    .map_err(|_| ToyError::ADCReadErr)?;

                Some(val)
            }

            None => None,
        };

        let adc_v: Option<u32> = match &mut self.adc_v_pin {
            Some(adc_pin) => {
                let val = self
                    .adc_handle
                    .read(adc_pin)
                    .map_err(|_| ToyError::ADCReadErr)?;

                Some(val)
            }

            None => None,
        };

        let adc_w: Option<u32> = match &mut self.adc_w_pin {
            Some(adc_pin) => {
                let val = self
                    .adc_handle
                    .read(adc_pin)
                    .map_err(|_| ToyError::ADCReadErr)?;

                Some(val)
            }

            None => None,
        };

        Ok((adc_u, adc_v, adc_w))
    }

    fn read_current(&mut self, angle_el: f32) -> nb::Result<(f32, f32), ToyError> {
        let (adc_u, adc_v, adc_w) = self.read_adc().map_err(|_| ToyError::ADCReadErr)?;

        let mut raw_u: f32 = match adc_u {
            Some(val) => val as f32 - self.offset_adc_u,
            None => 0.0,
        };

        let mut raw_v: f32 = match adc_v {
            Some(val) => val as f32 - self.offset_adc_v,
            None => 0.0,
        };

        let mut raw_w: f32 = match adc_w {
            Some(val) => val as f32 - self.offset_adc_w,
            None => 0.0,
        };

        if let None = adc_u {
            raw_u = -(raw_v + raw_w);
        }

        if let None = adc_v {
            raw_v = -(raw_u + raw_w);
        }

        if let None = adc_w {
            raw_w = -(raw_u + raw_v);
        }

        self.iu = raw_u * self.adc_conversion_factor * self.gain_u;
        self.iv = raw_v * self.adc_conversion_factor * self.gain_v;
        self.iw = raw_w * self.adc_conversion_factor * self.gain_w;

        self.cal_iq_id(angle_el);

        Ok(self.get_current())
    }

    fn cal_iq_id(&mut self, angle_el: f32) {
        let current_alpha = self.iu;
        let current_beta = _1_SQRT3 * self.iu + _2_SQRT3 * self.iv;

        let ct = _cos(angle_el);
        let st = _sin(angle_el);

        self.id = current_alpha * ct + current_beta * st;
        self.iq = -current_alpha * st + current_beta * ct;
    }

    fn get_phase_current(&mut self) -> PhaseCurrent {
        PhaseCurrent {
            u: self.iu,
            v: self.iv,
            w: self.iw,
        }
    }

    fn get_current(&mut self) -> (f32, f32) {
        (self.id, self.iq)
    }

    fn get_offset(&mut self) -> (f32, f32, f32) {
        (self.offset_adc_u, self.offset_adc_v, self.offset_adc_w)
    }
}
