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
