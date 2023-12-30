pub(crate) const _PI: f32 = 3.14159265359;
pub(crate) const _2PI: f32 = 6.28318530718;
pub(crate) const _3PI_2: f32 = 4.71238898038;
pub(crate) const _PI_3: f32 = 1.0471975512;
pub(crate) const _PI_2: f32 = 1.57079632679;
pub(crate) const _SQRT3: f32 = 1.73205080757;
pub(crate) const _1_SQRT3: f32 = 0.57735026919;
pub(crate) const _2_SQRT3: f32 = 1.15470053838;

pub fn _constrain(x: f32, min: f32, max: f32) -> f32 {
    if x < min {
        return min;
    }
    if x > max {
        return max;
    }
    return x;
}

// normalize angle to [0,2PI]
pub fn _normalize_angle(angle: f32) -> f32 {
    let a = angle % _2PI;
    if a >= 0.0 {
        a as f32
    } else {
        a as f32 + _2PI
    }
}

pub fn _electrical_angle(shaft_angle: f32, pole_pairs: u32) -> f32 {
    shaft_angle * (pole_pairs as f32)
}

pub fn _sin(a: f32) -> f32 {
    let sine_array: [u16; 65] = [
        0, 804, 1608, 2411, 3212, 4011, 4808, 5602, 6393, 7180, 7962, 8740, 9512, 10279, 11039,
        11793, 12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531, 18205, 18868, 19520, 20160,
        20788, 21403, 22006, 22595, 23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791, 27246,
        27684, 28106, 28511, 28899, 29269, 29622, 29957, 30274, 30572, 30853, 31114, 31357, 31581,
        31786, 31972, 32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758, 32768,
    ];
    let i = (a * (64.0 * 4.0 * 256.0 / _2PI)) as u32;

    let frac = (i & 0xff) as f32;

    let i = ((i >> 8) & 0xff) as usize;

    let (t1, t2) = if i < 64 {
        (sine_array[i] as f32, sine_array[i + 1] as f32)
    } else if i < 128 {
        (sine_array[128 - i] as f32, sine_array[127 - i] as f32)
    } else if i < 192 {
        (-(sine_array[i - 128] as f32), -(sine_array[i - 127] as f32))
    } else {
        (-(sine_array[256 - i] as f32), -(sine_array[255 - i] as f32))
    };

    (1.0 / 32768.0) * (t1 + ((t2 - t1) * frac) / 256.0)
}

pub fn _cos(a: f32) -> f32 {
    let mut a_sin = a + _PI_2;

    if a_sin > _2PI {
        a_sin = a_sin - _2PI
    }

    return _sin(a_sin);
}

pub fn _floor(x: f32) -> f32 {
    x - (x % 1.0) + if x < 0.0 { -1.0 } else { 0.0 }
}

pub fn _abs(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}
