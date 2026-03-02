pub trait StateVectors<P: Copy> {
    type Position;
    type Velocity;

    fn position(&self, param: P) -> Self::Position;

    fn velocity(&self, param: P) -> Self::Velocity;

    fn position_and_velocity(&self, param: P) -> (Self::Position, Self::Velocity) {
        (self.position(param), self.velocity(param))
    }
}
