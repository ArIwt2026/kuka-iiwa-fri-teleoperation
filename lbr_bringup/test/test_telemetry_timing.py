from lbr_bringup.telemetry_timing import NativeTimeMapper


def test_callback_jitter_does_not_change_native_sample_intervals():
    mapper = NativeTimeMapper()
    first = mapper.map(2_000_000_000, 10_000_000_000, 1_000_000_000)
    second = mapper.map(2_005_000_000, 10_020_000_000, 1_020_000_000)
    assert second.stamp_ns - first.stamp_ns == 5_000_000
    assert second.stamp_ns < 10_020_000_000


def test_duplicate_packet_does_not_acquire_a_new_timestamp():
    mapper = NativeTimeMapper()
    mapper.map(10, 100, 100)
    assert mapper.map(10, 200, 200) is None
    assert mapper.status == 'duplicate'


def test_reordered_packet_is_rejected_without_changing_clock_segment():
    mapper = NativeTimeMapper()
    mapper.map(2_000_000_000, 10_000_000_000, 1_000_000_000)
    assert mapper.map(1_995_000_000, 10_005_000_000, 1_005_000_000) is None
    assert mapper.status == 'out_of_order'
    next_sample = mapper.map(2_005_000_000, 10_010_000_000, 1_010_000_000)
    assert next_sample.segment == 1
    assert next_sample.stamp_ns == 10_005_000_000


def test_native_restart_creates_a_new_segment():
    mapper = NativeTimeMapper()
    mapper.map(20_000_000_000, 100_000_000_000, 1_000_000_000)
    sample = mapper.map(5_000_000, 100_005_000_000, 1_005_000_000)
    assert sample.segment == 2
    assert sample.reset_reason == 'native_clock_restart'
    assert sample.stamp_ns == 100_005_000_000


def test_host_clock_step_resets_mapping_even_with_duplicate_packets_in_between():
    mapper = NativeTimeMapper()
    mapper.map(2_000_000_000, 100_000_000_000, 1_000_000_000)
    assert mapper.map(2_000_000_000, 95_000_000_000, 1_005_000_000) is None
    sample = mapper.map(2_005_000_000, 95_005_000_000, 1_010_000_000)
    assert sample.reset_reason == 'host_clock_jump'
    assert sample.segment == 2


def test_better_offset_never_publishes_non_increasing_stamp():
    mapper = NativeTimeMapper()
    mapper.map(2_000_000_000, 100_000_000_000, 1_000_000_000)
    assert mapper.map(2_005_000_000, 100_000_000_000, 1_001_000_000) is None
    sample = mapper.map(2_010_000_000, 100_010_000_000, 1_010_000_000)
    assert sample.stamp_ns == 100_005_000_000


def test_delayed_sample_keeps_its_old_mapped_time():
    mapper = NativeTimeMapper()
    mapper.map(2_000_000_000, 100_000_000_000, 1_000_000_000)
    sample = mapper.map(2_005_000_000, 100_500_000_000, 1_500_000_000)
    assert sample.stamp_ns == 100_005_000_000


def test_invalid_times_are_rejected():
    mapper = NativeTimeMapper()
    assert mapper.map(-1, 100, 100) is None
    assert mapper.status == 'invalid_timestamp'
