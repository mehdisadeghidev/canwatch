// Per-ID frame statistics, kept out of the sketch so it can be checked on the host.
// No Arduino dependencies: test/frame_stats_test.cpp compiles this file directly.
#ifndef CANWATCH_FRAME_STATS_H
#define CANWATCH_FRAME_STATS_H

#include <stdint.h>
#include <string.h>

struct FrameStats {
  uint32_t pMin, pMax;     // period between frames, us (0xFFFFFFFF until sampled)
  uint64_t pSum;           // us, for the average
  uint16_t pN;
  uint32_t cMin, cMax;     // interval between payload changes, ms
  uint8_t orB[8], andB[8]; // per byte: bits ever seen 1 / bits never seen 0
  uint8_t togMask[8];      // per byte: bits that toggled at least once
  uint16_t nChg[8];        // per byte: payload changes touching that byte
  uint8_t cntUp[8];        // 16-bit window stepped by 1..8: 0-3 little endian, 4-7 big endian
};

static inline uint16_t statsSat16(uint16_t v) { return v == 0xFFFF ? v : (uint16_t)(v + 1); }
static inline uint8_t statsSat8(uint8_t v) { return v == 0xFF ? v : (uint8_t)(v + 1); }

// first sighting: the payload defines the baseline for every bit statistic
static inline void frameStatsInit(FrameStats &s, const uint8_t *data, uint8_t len) {
  memset(&s, 0, sizeof(s));
  s.pMin = 0xFFFFFFFF;
  s.cMin = 0xFFFFFFFF;
  for (uint8_t i = 0; i < len && i < 8; i++) s.orB[i] = s.andB[i] = data[i];
}

// every frame of a known ID. Gaps outside 0.5 ms .. 5 s are not period samples:
// event frames and buses that just woke up must not pollute the period stats.
static inline bool frameStatsPeriod(FrameStats &s, uint64_t dtUs) {
  if (dtUs < 500 || dtUs > 5000000ULL) return false;
  if (dtUs < s.pMin) s.pMin = (uint32_t)dtUs;
  if (dtUs > s.pMax) s.pMax = (uint32_t)dtUs;
  s.pSum += dtUs;
  if (s.pN < 0xFFFF) s.pN++;
  return true;
}

// payload changed. `prev` is the payload before this frame, `cur` the new one,
// `len` the new payload length, `chgDtMs` the gap since the previous change.
static inline void frameStatsChange(FrameStats &s, const uint8_t *cur, const uint8_t *prev,
                                    uint8_t len, uint32_t chgDtMs, bool hadChange) {
  if (hadChange) {
    if (chgDtMs < s.cMin) s.cMin = chgDtMs;
    if (chgDtMs > s.cMax) s.cMax = chgDtMs;
  }
  for (uint8_t i = 0; i < len && i < 8; i++) {
    const uint8_t x = (uint8_t)(prev[i] ^ cur[i]);
    s.togMask[i] |= x;
    s.orB[i] |= cur[i];
    s.andB[i] &= cur[i];
    if (x) s.nChg[i] = statsSat16(s.nChg[i]);
  }
  for (uint8_t w = 0; w < 4; w++) {
    if (2 * w + 1 >= len) break;
    const uint16_t leA = (uint16_t)(prev[2 * w] | (prev[2 * w + 1] << 8));
    const uint16_t leB = (uint16_t)(cur[2 * w] | (cur[2 * w + 1] << 8));
    const uint16_t dLe = (uint16_t)(leB - leA);
    if (dLe >= 1 && dLe <= 8) s.cntUp[w] = statsSat8(s.cntUp[w]);
    const uint16_t beA = (uint16_t)((prev[2 * w] << 8) | prev[2 * w + 1]);
    const uint16_t beB = (uint16_t)((cur[2 * w] << 8) | cur[2 * w + 1]);
    const uint16_t dBe = (uint16_t)(beB - beA);
    if (dBe >= 1 && dBe <= 8) s.cntUp[4 + w] = statsSat8(s.cntUp[4 + w]);
  }
}

#endif
