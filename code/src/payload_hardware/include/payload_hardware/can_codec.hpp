// can_codec.hpp -- bit-level packing of the 8-byte CAN data field.
//
// Data field layout (from CAN_Table.xlsx "Message Framework"), MSB first:
//
//   bit 0        : destination      (1 = Jetson, 0 = payload)
//   bits 1..6    : message id       (6 bits, up to 64 messages)
//   bits 7..63   : payload          (57 bits, fields defined per message)
//
// We treat the 8 bytes as ONE 64-bit big-endian word so that "bit n" in this file always
// means "n bits from the most significant end".  All of the bit arithmetic lives here so
// the rest of the code never shifts bits by hand.
#pragma once
#include <array>
#include <cstdint>

namespace payload_hardware {
namespace can {

constexpr int kHeaderBits = 7;       // destination + message id
constexpr int kPayloadBits = 57;
constexpr uint8_t kDestPayload = 0;
constexpr uint8_t kDestJetson = 1;

// A raw CAN frame as the hardware sees it.
struct Frame {
  uint32_t id = 0;                    // 29-bit extended arbitration id
  uint8_t dlc = 8;                    // number of data bytes
  std::array<uint8_t, 8> data{};
};

inline uint64_t mask_bits(int width) { return width >= 64 ? ~0ULL : ((1ULL << width) - 1ULL); }

// bytes -> word (byte 0 is the most significant byte)
inline uint64_t to_word(const Frame& f) {
  uint64_t w = 0;
  for (int i = 0; i < 8; ++i) w = (w << 8) | f.data[i];
  return w;
}

inline void from_word(Frame& f, uint64_t w) {
  for (int i = 7; i >= 0; --i) { f.data[i] = static_cast<uint8_t>(w & 0xFF); w >>= 8; }
  f.dlc = 8;
}

// Read `width` bits starting `start` bits from the MSB.  The first bit read is the MSB of the result.
inline uint64_t get_field(uint64_t w, int start, int width) {
  return (w >> (64 - (start + width))) & mask_bits(width);
}

// Write `value` (truncated to `width` bits) into the same position.
inline uint64_t set_field(uint64_t w, int start, int width, uint64_t value) {
  const int shift = 64 - (start + width);
  w &= ~(mask_bits(width) << shift);
  return w | ((value & mask_bits(width)) << shift);
}

// Two's complement: interpret the low `width` bits of v as a signed number.
inline int64_t sign_extend(uint64_t v, int width) {
  const uint64_t sign = 1ULL << (width - 1);
  v &= mask_bits(width);
  return static_cast<int64_t>((v ^ sign)) - static_cast<int64_t>(sign);
}

// ---- header helpers ----
inline uint8_t get_dest(uint64_t w) { return static_cast<uint8_t>(get_field(w, 0, 1)); }
inline uint8_t get_msg_id(uint64_t w) { return static_cast<uint8_t>(get_field(w, 1, 6)); }
inline uint64_t make_header(uint8_t dest, uint8_t msg_id) {
  uint64_t w = 0;
  w = set_field(w, 0, 1, dest);
  w = set_field(w, 1, 6, msg_id);
  return w;
}

// ---- payload helpers: `bit` is relative to the first payload bit ----
inline uint64_t get_payload(uint64_t w, int bit, int width) { return get_field(w, kHeaderBits + bit, width); }
inline uint64_t put_payload(uint64_t w, int bit, int width, uint64_t v) {
  return set_field(w, kHeaderBits + bit, width, v);
}

}  // namespace can
}  // namespace payload_hardware
