"""Map native FRI time to ROS time without refreshing repeated measurements.

The offset is the lowest observed host-callback minus native timestamp in a clock
segment. It includes unknown transport/startup delay; this is a software estimate,
not a measured synchronization of the robot and host clocks.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class MappedSample:
    native_ns: int
    stamp_ns: int
    segment: int
    reset_reason: str = ''


class NativeTimeMapper:
    def __init__(self, clock_jump_ns=100_000_000, restart_ns=1_000_000_000):
        self.clock_jump_ns = clock_jump_ns
        self.restart_ns = restart_ns
        self.segment = 0
        self.offset_ns = None
        self.last_native_ns = None
        self.last_ros_ns = None
        self.last_steady_ns = None
        self.last_stamp_ns = None
        self.status = 'uninitialized'

    def map(self, native_ns, ros_ns, steady_ns):
        if native_ns < 0 or ros_ns < 0 or steady_ns < 0:
            self.status = 'invalid_timestamp'
            return None

        reason = ''
        if self.last_steady_ns is not None:
            elapsed = steady_ns - self.last_steady_ns
            if elapsed < 0 or abs((ros_ns - self.last_ros_ns) - elapsed) > self.clock_jump_ns:
                reason = 'host_clock_jump'
            elif native_ns < self.last_native_ns:
                if self.last_native_ns - native_ns >= self.restart_ns or elapsed >= self.restart_ns:
                    reason = 'native_clock_restart'
                else:
                    self.status = 'out_of_order'
                    return None
            elif native_ns - self.last_native_ns - elapsed > self.restart_ns:
                reason = 'native_clock_jump'

        # Even after a host clock step, a repeated robot packet is not new feedback.
        if native_ns == self.last_native_ns:
            self.status = 'duplicate'
            return None

        if self.offset_ns is None or reason:
            self.segment += 1
            self.offset_ns = ros_ns - native_ns
            self.last_stamp_ns = None
        else:
            self.offset_ns = min(self.offset_ns, ros_ns - native_ns)

        stamp_ns = native_ns + self.offset_ns
        self.last_native_ns = native_ns
        self.last_ros_ns = ros_ns
        self.last_steady_ns = steady_ns
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.status = 'non_increasing_mapped_time'
            return None
        self.last_stamp_ns = stamp_ns
        self.status = reason or 'mapped'
        return MappedSample(native_ns, stamp_ns, self.segment, reason)
