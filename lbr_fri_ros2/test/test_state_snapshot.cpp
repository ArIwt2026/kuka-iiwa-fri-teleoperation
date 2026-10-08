#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "lbr_fri_idl/msg/lbr_state.hpp"
#include "lbr_fri_ros2/state_snapshot.hpp"

using State = lbr_fri_idl::msg::LBRState;

namespace {
State packet(uint32_t generation) {
  State state;
  state.time_stamp_sec = generation;
  state.time_stamp_nano_sec = generation;
  state.measured_joint_position.fill(generation);
  state.commanded_joint_position.fill(generation);
  state.measured_torque.fill(generation);
  state.commanded_torque.fill(generation);
  state.external_torque.fill(generation);
  state.ipo_joint_position.fill(generation);
  state.sample_time = generation;
  return state;
}

bool coherent(const State &state) {
  const auto generation = state.time_stamp_sec;
  if (state.time_stamp_nano_sec != generation || state.sample_time != generation) return false;
  for (std::size_t i = 0; i < 7; ++i) {
    if (state.measured_joint_position[i] != generation ||
        state.commanded_joint_position[i] != generation || state.measured_torque[i] != generation ||
        state.commanded_torque[i] != generation || state.external_torque[i] != generation ||
        state.ipo_joint_position[i] != generation) return false;
  }
  return true;
}
} // namespace

TEST(StateSnapshot, ReturnedCopySurvivesFurtherPublications) {
  lbr_fri_ros2::StateSnapshot<State> snapshot;
  snapshot.publish(packet(1));
  const auto saved = snapshot.read();
  for (uint32_t i = 2; i < 100; ++i) snapshot.publish(packet(i));
  EXPECT_EQ(saved.time_stamp_sec, 1u);
  EXPECT_TRUE(coherent(saved));
  const auto latest = snapshot.read();
  EXPECT_EQ(latest.time_stamp_sec, 99u);
  EXPECT_TRUE(coherent(latest));
}

TEST(StateSnapshot, ConcurrentReadersNeverObserveMixedPackets) {
  lbr_fri_ros2::StateSnapshot<State> snapshot;
  std::atomic<bool> start{false}, done{false};
  std::atomic<uint32_t> failures{0}, reads{0};
  std::vector<std::thread> readers;
  for (int i = 0; i < 3; ++i) {
    readers.emplace_back([&] {
      while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
      uint32_t previous = 0;
      do {
        const auto state = snapshot.read();
        if (!coherent(state) || state.time_stamp_sec < previous) ++failures;
        previous = state.time_stamp_sec;
        ++reads;
      } while (!done.load(std::memory_order_acquire));
    });
  }
  start.store(true, std::memory_order_release);
  for (uint32_t i = 1; i <= 200000; ++i) snapshot.publish(packet(i));
  done.store(true, std::memory_order_release);
  for (auto &reader : readers) reader.join();
  EXPECT_GT(reads.load(), 0u);
  EXPECT_EQ(failures.load(), 0u);
  EXPECT_EQ(snapshot.read().time_stamp_sec, 200000u);
}
