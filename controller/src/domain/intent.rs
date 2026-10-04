#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionIntent {
    pub name: &'static str,
    pub left_mm_s: i16,
    pub right_mm_s: i16,
}

impl MotionIntent {
    pub const fn new(name: &'static str, left_mm_s: i16, right_mm_s: i16) -> Self {
        Self {
            name,
            left_mm_s,
            right_mm_s,
        }
    }
}
