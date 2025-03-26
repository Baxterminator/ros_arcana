#ifndef ARCANA_CIRCULAR_BUFFER_HPP
#define ARCANA_CIRCULAR_BUFFER_HPP

#include <cstddef>
#include <sstream>
#include <string>

namespace arcana {

/**
  Circular buffer implementation
*/
template <typename T, size_t N> struct CircularBuffer {

  /**
    Get the size of the array
  */
  size_t size() const { return size_counter; }

  /**
    Check if the buffer is full.
    If buffer full, front = back - 1
  */
  inline bool is_full() const { return size_counter == N; }

  /**
    Check if the buffer is empty.
    If buffer empty, back = front
  */
  inline bool is_empty() const { return size_counter == 0; }

  /**
    Push a new value in the buffer.
    If the buffer is full, overwrite the old values
  */
  void push(const T &v) {
    if (is_full())
      inc_tail();

    inc_head();
    size_counter++;
    buffer[head] = v;
  }

  T pop() {
    if (is_empty())
      return 0.0;

    T elem = buffer[tail];
    inc_tail();
    size_counter--;
    return elem;
  }

  void flush() {
    size_counter = 0;
    head = 0;
    tail = 0;
  }

protected:
  void inc_head() { head = (head + 1) % N; }
  void inc_tail() { tail = (tail + 1) % N; }

  // ==========================================================================
  // Buffer access
  // ==========================================================================
public:
  const size_t get_head_idx() const { return head; }
  const size_t get_tail_idx() const { return tail; }
  const T front() const { return buffer[head]; }
  inline const T &raw_get(const size_t &idx) const { return buffer[idx % N]; }
  inline const T &get(const size_t &idx) const {
    return buffer[(tail + idx) % N];
  }

  // ==========================================================================
  // Serialization
  // ==========================================================================
protected:
  inline const std::string serialize_offset(size_t offset, size_t n = N) const {
    std::stringstream ss;
    ss << "CircBuffer[";
    const auto max_N = std::min(n, N);
    for (size_t i = 0; i < max_N; i++)
      ss << buffer[(offset + i) % N] << ((i == max_N - 1) ? "" : ", ");
    ss << "]";
    return ss.str();
  }

public:
  const std::string serialize_raw() const { return serialize_offset(0); }
  const std::string serialize() const {
    // return serialize_offset((tail + 1) % N, size());
    return serialize_raw();
  }

protected:
  T buffer[N];

  // Head is where the latest value has been placed
  size_t head = 0;
  // Tail is where the oldest value has been placed
  size_t tail = 0;
  // Size_counter is the number of elements in the buffer
  size_t size_counter = 0;
};

} // namespace arcana

#endif