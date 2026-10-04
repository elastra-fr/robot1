#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionHealth {
    Starting,
    Connected,
    Recovering,
}
