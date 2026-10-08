#ifndef LBR_FRI_ROS2__STATE_SNAPSHOT_HPP_
#define LBR_FRI_ROS2__STATE_SNAPSHOT_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <type_traits>

namespace lbr_fri_ros2 {

// One serialized FRI producer; readers serialize only with each other. Slot
// ownership prevents the producer from overwriting a snapshot being copied.
// Publishing never waits for readers and must use a bounded, allocation-free T.
template <typename T> class StateSnapshot {
public:
  static_assert(std::is_copy_assignable<T>::value, "Snapshot must be copyable");
  static_assert(std::is_nothrow_copy_assignable<T>::value, "Publishing must not throw");
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "Slot exchange must be lock-free");

  void publish(const T &value) noexcept {
    slots_[producer_slot_] = value;
    producer_slot_ = exchange_.exchange(producer_slot_ | kUnread, std::memory_order_acq_rel) & kIndex;
  }

  T read() const {
    std::lock_guard<std::mutex> lock(reader_mutex_);
    if (exchange_.load(std::memory_order_acquire) & kUnread) {
      consumer_slot_ = exchange_.exchange(consumer_slot_, std::memory_order_acq_rel) & kIndex;
    }
    return slots_[consumer_slot_];
  }

private:
  static constexpr uint32_t kUnread = 4;
  static constexpr uint32_t kIndex = 3;
  std::array<T, 3> slots_{{T{}, T{}, T{}}};
  uint32_t producer_slot_{2};
  mutable uint32_t consumer_slot_{0};
  mutable std::atomic<uint32_t> exchange_{1};
  mutable std::mutex reader_mutex_;
};

} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__STATE_SNAPSHOT_HPP_
