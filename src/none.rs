use crate::{current::CurrentSensorTrait, PhaseCurrent, ToyError};
use embedded_hal::{adc::Channel, blocking::delay::DelayUs, digital::v2::OutputPin};
use mt6701::AngleSensorTrait;

pub const NONE_ENABLE_PIN: Option<NoneEnablePin> = Option::<NoneEnablePin>::None;

pub const NONE_SENSOR: Option<NoneSensor> = Option::<NoneSensor>::None;

pub const NONE_ADC_PIN: Option<NoneAdcPin> = Option::<NoneAdcPin>::None;
pub const NONE_CURRENT_SENSOR: Option<NoneCurrentSensor> = Option::<NoneCurrentSensor>::None;

pub struct NoneEnablePin {}
impl OutputPin for NoneEnablePin {
    type Error = ToyError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct NoneSensor {}
impl AngleSensorTrait for NoneSensor {
    type Error = ToyError;

    fn init(&mut self) -> nb::Result<(), ToyError> {
        Ok(())
    }

    fn read_raw_angle(&mut self) -> nb::Result<u16, ToyError> {
        Ok(0)
    }

    fn update(&mut self, _ts_ns: u64) -> nb::Result<(), ToyError> {
        Ok(())
    }

    fn get_angle(&mut self) -> f32 {
        0.0
    }

    fn get_turns(&mut self) -> i64 {
        0
    }

    fn get_position(&mut self) -> f64 {
        0.0
    }

    fn get_velocity(&mut self) -> f32 {
        0.0
    }
}

pub struct NoneAdcPin {}
impl<ADC> Channel<ADC> for NoneAdcPin {
    type ID = u8;

    fn channel() -> Self::ID {
        u8::MAX
    }
}

pub struct NoneCurrentSensor {}
impl CurrentSensorTrait for NoneCurrentSensor {
    fn init<D>(&mut self, _delay: &mut D) -> nb::Result<(), ToyError>
    where
        D: DelayUs<u32>,
    {
        Ok(())
    }

    fn read_adc(&mut self) -> nb::Result<(Option<u32>, Option<u32>, Option<u32>), ToyError> {
        Ok((None, None, None))
    }

    fn read_current(&mut self, _angle_el: f32) -> nb::Result<(f32, f32), ToyError> {
        Ok((0.0, 0.0))
    }

    fn cal_iq_id(&mut self, _angle_el: f32) {}

    fn get_phase_current(&mut self) -> PhaseCurrent {
        PhaseCurrent {
            u: 0.0,
            v: 0.0,
            w: 0.0,
        }
    }

    fn get_current(&mut self) -> (f32, f32) {
        (0.0, 0.0)
    }

    fn get_offset(&mut self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }
}
