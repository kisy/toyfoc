use crate::utils::_constrain;

// include pid control and low pass filter
pub struct PID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub val: f32,
    integral_prev: f32,
    prev_us: u64,
    error_prev: f32,
    pub limit: f32,
    output_prev: f32,
    pub output_ramp: f32,
    pub filter_val: f32,
    filter_prev: f32,
    pub filter_tf: f32,
    filter_us: u64,
}

impl PID {
    pub fn new(p: f32, i: f32, d: f32, output_ramp: f32, output_limit: f32, tf: f32) -> Self {
        PID {
            p,
            i,
            d,
            val: 0.0,
            integral_prev: 0.0,
            prev_us: 0,
            error_prev: 0.0,
            limit: output_limit,
            output_prev: 0.0,
            output_ramp,
            filter_val: 0.0,
            filter_prev: 0.0,
            filter_tf: tf,
            filter_us: 0,
        }
    }

    pub fn pid(&mut self, error: f32, now_us: u64) -> f32 {
        let mut ts = (now_us - self.prev_us) as f32 * 1e-6;
        if ts < 0.0 {
            ts = 1e-3;
        }

        let proportional = self.p * error;

        let mut integral = self.integral_prev + self.i * ts * 0.5 * (error + self.error_prev);
        integral = _constrain(integral, -self.limit, self.limit);

        let derivative = self.d * (error - self.error_prev) / ts;

        let mut output = proportional + integral + derivative;

        output = _constrain(output, -self.limit, self.limit);

        if self.output_ramp > 0.0 {
            let output_rate = (output - self.output_prev) / ts;
            if output_rate > self.output_ramp {
                output = self.output_prev + self.output_ramp * ts;
            } else if output_rate < -self.output_ramp {
                output = self.output_prev - self.output_ramp * ts;
            }
        }

        self.output_prev = output;
        self.integral_prev = integral;
        self.error_prev = error;
        self.prev_us = now_us;

        output
    }

    pub fn low_pass_filter(&mut self, input: f32, now_us: u64) -> f32 {
        let mut ts = (now_us - self.filter_us) as f32 * 1e-6;

        if ts < 0.0 {
            ts = 1e-3;
        } else if ts > 0.3 {
            self.filter_prev = input;
            self.filter_us = now_us;
            return input;
        }

        let alpha = self.filter_tf / (self.filter_tf + ts);
        let output = alpha * self.filter_prev + (1.0 - alpha) * input;
        self.filter_prev = output;
        self.filter_us = now_us;

        output
    }

    pub fn reset(&mut self) {
        self.integral_prev = 0.0;
        self.error_prev = 0.0;
        self.output_prev = 0.0;
        self.filter_prev = 0.0;
        self.filter_us = 0;
        self.prev_us = 0;
    }
}
