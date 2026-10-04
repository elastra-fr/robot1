use std::time::{Duration, Instant};

pub(super) struct ConnectionSupervisor {
    heartbeat_interval: Duration,
    heartbeat_deadline: Instant,
}

impl ConnectionSupervisor {
    pub(super) fn new(heartbeat_interval: Duration, now: Instant) -> Self {
        Self {
            heartbeat_interval,
            heartbeat_deadline: now,
        }
    }

    pub(super) fn heartbeat_due(&mut self, now: Instant) -> bool {
        if now < self.heartbeat_deadline {
            return false;
        }

        self.heartbeat_deadline = now + self.heartbeat_interval;
        true
    }

    pub(super) fn reset_after_recovery(&mut self, now: Instant) {
        self.heartbeat_deadline = now + self.heartbeat_interval;
    }
}
