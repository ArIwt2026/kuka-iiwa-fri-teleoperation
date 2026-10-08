"""Exercise the actual ROS publishers in a separate DDS domain and namespace."""
import importlib.machinery
import importlib.util
import math
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node
from sensor_msgs.msg import JointState


def test_ros_bridge_preserves_native_timing_and_packet_values(monkeypatch):
    script = Path(__file__).parents[1] / 'scripts/admittance_telemetry_bridge'
    loader = importlib.machinery.SourceFileLoader('bridge_under_test', str(script))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    clocks = {'ros': 1_700_000_000_000_000_000, 'steady': 1_000_000_000}
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic_ns=lambda: clocks['steady']))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/kuka_sync_synthetic_test'])
    bridge = module.AdmittanceTelemetryBridge()
    bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clocks['ros']))
    observer = Node('observer')
    messages = {'raw': [], 'measured': [], 'commanded': [], 'timing': []}
    subscriptions = [observer.create_subscription(kind, topic, messages[key].append, 100)
                     for key, kind, topic in [
                         ('raw', LBRState, 'fri_state'),
                         ('measured', JointState, 'joint_states'),
                         ('commanded', JointState, 'joint_commands'),
                         ('timing', DiagnosticArray, 'telemetry_timing')]]
    publisher = observer.create_publisher(LBRState, 'state', 100)
    command_publisher = observer.create_publisher(JointState, 'admittance_controller/joint_commands', 100)

    def spin_until(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(bridge, timeout_sec=0.005)
            rclpy.spin_once(observer, timeout_sec=0.005)
        assert predicate(), {key: len(value) for key, value in messages.items()}

    def send(native_ns, q, ros_elapsed_ns, raw_count):
        clocks['ros'] += ros_elapsed_ns
        clocks['steady'] += ros_elapsed_ns
        state = LBRState()
        state.time_stamp_sec, state.time_stamp_nano_sec = divmod(native_ns, 1_000_000_000)
        state.sample_time = 0.005
        state.measured_joint_position = [q] * 7
        state.measured_torque = [2.0] * 7
        state.commanded_joint_position = [q + 0.1] * 7
        state.commanded_torque = [3.0] * 7
        publisher.publish(state)
        spin_until(lambda: len(messages['raw']) >= raw_count)

    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    try:
        spin_until(lambda: publisher.get_subscription_count() == 1 and
                   command_publisher.get_subscription_count() == 1 and
                   all(observer.count_publishers(sub.topic_name) == 1 for sub in subscriptions))
        send(2_000_000_000, 0.0, 0, 1)
        spin_until(lambda: len(messages['timing']) == 1 and len(messages['measured']) == 1)
        assert list(messages['measured'][0].velocity) == []
        send(2_005_000_000, 0.01, 20_000_000, 2)
        spin_until(lambda: len(messages['timing']) == 2 and len(messages['measured']) == 2
                   )
        first, second = messages['measured']
        assert stamp(second) - stamp(first) == 5_000_000
        assert stamp(second) == stamp(messages['timing'][1])
        assert list(second.position) == [0.01] * 7
        assert all(math.isclose(v, 2.0) for v in second.velocity)
        assert list(second.effort) == [2.0] * 7
        values = {v.key: v.value for v in messages['timing'][1].status[0].values}
        assert int(values['native_ns']) == 2_005_000_000
        assert int(values['estimated_age_ns']) == 15_000_000

        assert messages['commanded'] == []  # Native FRI command fields are not the overlay targets.
        command = JointState()
        command.header.stamp.sec, command.header.stamp.nanosec = divmod(
            clocks['ros'] + 1_000_000, 1_000_000_000)
        command.name = [f'lbr_A{i}' for i in range(1, 8)]
        command.position = [0.9] * 7
        command_publisher.publish(command)
        spin_until(lambda: len(messages['commanded']) == 1)
        forwarded = messages['commanded'][0]
        assert list(forwarded.name) == [f'iiwa7_A{i}' for i in range(1, 8)]
        assert list(forwarded.position) == [0.9] * 7
        assert forwarded.header == command.header

        send(2_005_000_000, 0.01, 5_000_000, 3)  # repeated hardware packet
        send(2_000_000_000, 0.0, 5_000_000, 4)  # reordered hardware packet
        send(2_010_000_000, 0.02, 200_000_000, 5)  # delayed genuine packet
        # The raw copy is queued before the derived messages, so drain callbacks.
        for _ in range(10):
            rclpy.spin_once(observer, timeout_sec=0.005)
        assert len(messages['measured']) == len(messages['timing']) == 2
        assert len(messages['commanded']) == 1

        send(5_000_000, 0.03, 5_000_000, 6)  # native clock restarted
        spin_until(lambda: len(messages['timing']) == 3 and len(messages['measured']) == 3)
        assert list(messages['measured'][-1].velocity) == []
        values = {v.key: v.value for v in messages['timing'][-1].status[0].values}
        assert values['mapping_reset'] == 'native_clock_restart'
        assert int(values['clock_segment']) == 2
    finally:
        observer.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
