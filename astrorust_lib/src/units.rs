mod units;

pub type MuDimension = uom::si::ISQ<P3, Z0, N2, Z0, Z0, Z0, Z0, dyn uom::Kind>;
pub type StdGravParam<U, V> = Quantity<MuDimension, U, V>;
