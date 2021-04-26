use dimensioned::{si::SI, typenum::{P3, Z0, N2}};

pub type StdGravParam<V> = SI<V, tarr![P3, Z0, N2, Z0, Z0, Z0, Z0]>;
