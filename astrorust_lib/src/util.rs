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
