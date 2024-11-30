pub trait StateVectors<P: Copy>: StateVectorTypes {
    fn position(&self, param: P) -> Self::Position;

    fn velocity(&self, param: P) -> Self::Velocity;

    fn position_and_velocity(&self, param: P) -> (Self::Position, Self::Velocity) {
        (self.position(param), self.velocity(param))
    }
}

pub trait StateVectorTypes {
    type Position;
    type Velocity;
}
