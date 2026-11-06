// Host-side check for canwatch_frame_stats.h:  g++ -std=c++11 -Wall -o t test/frame_stats_test.cpp && ./t
#include <assert.h>
#include <stdio.h>

#include "../canwatch_frame_stats.h"

// feed one payload; hadChange/prev handling mirrors handleFrame()
struct Feed {
  FrameStats st;
  uint8_t prev[8];
  uint8_t len = 0;
  uint32_t changes = 0;
  void first(const uint8_t *d, uint8_t n) {
    len = n;
    memcpy(prev, d, 8);
    frameStatsInit(st, d, n);
  }
  void next(const uint8_t *d, uint64_t dtUs, uint32_t chgDtMs) {
    frameStatsPeriod(st, dtUs);
    if (memcmp(prev, d, len) == 0) return;
    frameStatsChange(st, d, prev, len, chgDtMs, changes != 0);
    changes++;
    memcpy(prev, d, 8);
  }
};

int main() {
  // --- static ID: nothing toggles, period stats still build up ---
  {
    Feed f;
    f.first((const uint8_t[]){0xAA, 0x55, 0x00, 0xFF, 1, 2, 3, 4}, 8);
    for (int i = 0; i < 5; i++) f.next(f.prev, 20000, 0);
    assert(f.changes == 0);
    assert(f.st.pN == 5 && f.st.pMin == 20000 && f.st.pMax == 20000);
    assert(f.st.pSum / f.st.pN == 20000);
    for (int i = 0; i < 8; i++) {
      assert(f.st.nChg[i] == 0);
      assert(f.st.togMask[i] == 0);
      assert(f.st.orB[i] == f.st.andB[i]);
    }
  }

  // --- period outliers and sub-ms gaps are not samples ---
  {
    Feed f;
    f.first((const uint8_t[]){0}, 1);
    assert(!frameStatsPeriod(f.st, 100));        // 100 us: too short
    assert(!frameStatsPeriod(f.st, 10000000));   // 10 s: caught up after a pause
    assert(frameStatsPeriod(f.st, 1000));        // 1 ms: kept
    assert(f.st.pN == 1);
  }

  // --- little-endian counter in D2..D3 ---
  {
    Feed f;
    uint8_t d[8] = {0, 0, 0x00, 0x01, 0, 0, 0, 0};
    f.first(d, 8);
    for (int i = 0; i < 9; i++) {
      d[2]++;
      if (d[2] == 0) d[3]++;
      f.next(d, 10000, 10);
    }
    assert(f.st.cntUp[1] == 9);       // LE window on bytes 2..3
    assert(f.st.cntUp[5] == 0);       // BE window on the same bytes
    assert(f.st.nChg[2] == 9 && f.st.nChg[3] == 0);   // D3 never moved
    assert(f.st.nChg[0] == 0);        // constant byte
    assert(f.st.cMin == 10 && f.st.cMax == 10);
    assert(f.changes == 9);
  }

  // --- counter wrap 0xFFFF -> 0x0000 still steps by one ---
  {
    Feed f;
    uint8_t d[8] = {0, 0, 0xFF, 0xFF, 0, 0, 0, 0};
    f.first(d, 8);
    d[2] = 0; d[3] = 0;
    f.next(d, 10000, 5);
    assert(f.st.cntUp[1] == 1);
  }

  // --- single bit toggles, and what stays constant ---
  {
    Feed f;
    uint8_t d[8] = {0x00, 0xFF, 0x10, 0, 0, 0, 0, 0};
    f.first(d, 8);
    d[0] = 0x01;
    f.next(d, 10000, 7);
    d[0] = 0x00;
    f.next(d, 10000, 9);
    assert(f.st.togMask[0] == 0x01);            // only bit0 ever moved
    assert(f.st.orB[0] == 0x01 && f.st.andB[0] == 0x00);
    assert(f.st.orB[1] == 0xFF && f.st.andB[1] == 0xFF);   // byte 1 always FF
    assert(f.st.nChg[0] == 2 && f.st.nChg[1] == 0);
    assert(f.st.cMin == 9 && f.st.cMax == 9);   // first change has no gap to measure
  }

  // --- short payload: windows beyond the length are untouched ---
  {
    Feed f;
    uint8_t d[8] = {1, 2, 3, 0, 0, 0, 0, 0};
    f.first(d, 3);
    d[0] = 2;
    f.next(d, 10000, 4);
    assert(f.st.cntUp[0] == 1);                 // D0..D1 in range
    assert(f.st.cntUp[1] == 0);                 // D2..D3 out of range
  }

  printf("frame_stats: all checks passed\n");
  return 0;
}
