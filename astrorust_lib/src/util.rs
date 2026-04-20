use nalgebra::Point3;

pub trait FloatExt: Sized {
    fn sinh_cosh(self) -> (Self, Self);
}

impl FloatExt for f64 {
    fn sinh_cosh(self) -> (Self, Self) {
        let exp = self.exp();
        let exp_recip = exp.recip();
        ((exp - exp_recip) * 0.5, (exp + exp_recip) * 0.5)
    }
}

pub fn format_with_thousand_separators(mut num: u64) -> String {
    let mut n = num;
    let mut digits = 1usize;
    while n >= 10 {
        n /= 10;
        digits += 1;
    }

    let separators = (digits - 1) / 3;
    let total_len = digits + separators;
    let mut bytes = vec![0u8; total_len];

    let mut group_digits = 0usize;

    for idx in (0..total_len).rev() {
        if group_digits == 3 {
            bytes[idx] = b'\'';
            group_digits = 0;
        } else {
            bytes[idx] = b'0' + (num % 10) as u8;
            num /= 10;
            group_digits += 1;
        }
    }

    String::from_utf8(bytes).expect("only ASCII digits and separators")
}


/// Solve `a·x² + b·x + c = 0` for real roots, returning `(x_plus, x_minus)`
/// with `x_plus ≥ x_minus`. Returns `None` if the discriminant is negative
/// or the leading coefficient is degenerate.
pub fn solve_quadratic(a: f64, b: f64, c: f64) -> Option<(f64, f64)> {
    if a.abs() < 1e-24 {
        return None;
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let inv_2a = 0.5 / a;
    let x1 = (-b + sqrt_disc) * inv_2a;
    let x2 = (-b - sqrt_disc) * inv_2a;
    if x1 >= x2 { Some((x1, x2)) } else { Some((x2, x1)) }
}

pub fn rgb8_to_color([r, g, b]: [u8; 3]) -> Point3<f32> {
    Point3::<f32>::new(r.into(), g.into(), b.into()) / 255.0
}

