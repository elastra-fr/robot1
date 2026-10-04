use crate::domain::ActuatorCommand;

pub trait MotionOutput {
    type Error;

    fn apply(&mut self, command: ActuatorCommand) -> Result<u16, Self::Error>;
}
