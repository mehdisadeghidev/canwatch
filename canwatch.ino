#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ACAN2515.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include "canwatch_frame_stats.h"

// ---------- Settings ----------
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;   // 16 if crystal says 16.000

// ---------- Station ----------
static const bool STA_ENABLE = false;      // join a WiFi network before falling back to the AP
static const bool STA_USE_DHCP = true;    // true = router assigns the address; false = static below
static const char* STA_SSID     = "";
static const char* STA_USERNAME = "";     // WPA2-Enterprise identity; empty = WPA2 personal
static const char* STA_PASSWORD = "";
static const char* STA_IP       = "10.10.90.2";   // must sit inside the router LAN, never the AP address
static const char* STA_GATEWAY  = "10.10.90.1";
static const char* STA_SUBNET   = "255.255.255.0";   // /24, same mask as the router
static const uint8_t STA_TIMEOUT_SEC = 10;

// ---------- Access point ----------
static const char* AP_SSID     = "Canwatch";
static const char* AP_PASSWORD = "";     // 8+ chars = WPA2; empty or shorter = open network
static const char* AP_IP       = "10.10.98.1";
static const char* AP_GATEWAY  = "10.10.98.1";
static const char* AP_SUBNET   = "255.255.255.0";

// selectable from web page, 125k default
static const uint32_t BITRATES[] = {10000, 20000, 50000, 100000, 125000, 250000, 500000, 1000000};
static const uint8_t BITRATE_COUNT = sizeof(BITRATES) / sizeof(BITRATES[0]);
// ------------------------------

static const byte MCP2515_CS = 26;
// Wire MCP2515 INT to this GPIO and set it: frames are then captured by interrupt
// instead of polling, and nothing is lost while the web server is busy.
// 255 = keep polling (INT left unconnected).
static const byte MCP2515_INT = 255;
static const byte PIN_SCK = 14, PIN_MISO = 27, PIN_MOSI = 13;

// Bus tools: compile-time gate for anything that transmits (OBD-II / UDS / raw).
// The runtime gate is off at boot and needs a deliberate click in the UI.
static const bool ALLOW_TX = true;

static const int MAX_IDS = 128;
static const int LOG_SIZE = 60;
static const int RAW_SIZE = 384;          // raw frame ring: 384 * ~24 B, ~9 KB

ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);   // 255 = INT unused, we poll
WebServer server(80);

static bool listenOnly = true;        // default: listen only, never ACK/transmit
static uint32_t bitrate = 125000;
static uint32_t frameTotal = 0;

// ---------- Time ----------
// (declared further down: the Arduino IDE inserts generated prototypes just before
// the first function, so every struct used in a signature must be declared above it)

// ---------- Data tables ----------
struct Entry {
  bool used, ext, rtr;
  uint32_t id;
  uint8_t len, prevLen;
  uint8_t data[8], prev[8];
  uint32_t count, changes;
  uint64_t firstSeen, lastSeen, lastChange;
  FrameStats st;            // period, bit map, counter hints (canwatch_frame_stats.h)
};
static Entry table_[MAX_IDS];

struct LogLine {
  uint64_t t;
  uint32_t id;
  bool ext, isNew;
  uint8_t len, prevLen;
  uint8_t data[8], prev[8];
};
static LogLine logBuf[LOG_SIZE];
static int logHead = 0;
static int logCount = 0;

struct Raw {
  uint64_t tUs;
  uint32_t id;
  uint8_t len, flags, data[8];   // flags: bit0 ext, bit1 rtr
};
static Raw rawBuf[RAW_SIZE];
static int rawHead = 0;
static int rawCount = 0;

// ---------- Bus health ----------
struct Health {
  uint8_t rec, tec, eflg, eflgSeen;
  uint8_t brp, tq, sjw, cnf1, cnf2, cnf3;   // MCP2515 bit timing actually programmed
  uint32_t abr;            // actual bit rate of the active timing
  uint16_t ppm;            // timing error vs requested rate
  uint16_t sp;             // sample point, percent
  uint16_t rxPeak, rxSize;
  uint32_t droppedIds, rtrFrames;
  uint64_t bits;           // estimated bits seen since healthBaseUs
} health;
static uint64_t healthBaseUs = 0;

// ---------- JSON writer ----------
// Every struct used in a function signature has to be declared above the first
// function: the Arduino IDE inserts its generated prototypes right before it.
static char outBuf[40960];

struct Out {
  char *b;
  size_t cap, n;
  bool ov;
  void ch(char c) { if (n + 1 < cap) b[n++] = c; else ov = true; }
  void raw(const char *s) { while (*s) ch(*s++); }
  void num(uint64_t v) {
    char t[22];
    uint8_t i = 0;
    if (!v) t[i++] = '0';
    while (v) { t[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i) ch(t[--i]);
  }
  void f2(float v) {
    char t[24];
    snprintf(t, sizeof(t), "%.2f", (double)v);
    raw(t);
  }
  void hex(const uint8_t *d, uint8_t l) {
    static const char H[] = "0123456789ABCDEF";
    for (uint8_t i = 0; i < l; i++) { ch(H[d[i] >> 4]); ch(H[d[i] & 15]); }
  }
  void key(const char *k) { ch('"'); raw(k); raw("\":"); }
  void end() { if (n < cap) b[n] = 0; }
};

static const size_t OUT_LIMIT = sizeof(outBuf) - 64;   // room for the closing braces

// ---------- Time ----------
// 64-bit microseconds since boot: no millis() wrap after 49 days.
uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }
uint64_t nowMs() { return nowUs() / 1000ULL; }

uint16_t frameBits(bool ext, uint8_t len) {
  const uint16_t core = ext ? 67 : 47;         // SOF..IFS overhead, no data, no stuffing
  const uint16_t b = core + 8u * len;
  return (uint16_t)(b + b / 5);                // + ~20% average stuff bits
}

// ---------- Receive path ----------
void addRaw(const CANMessage &f, uint64_t t) {
  Raw &r = rawBuf[rawHead];
  r.tUs = t;
  r.id = f.id;
  r.len = f.len;
  r.flags = (f.ext ? 1 : 0) | (f.rtr ? 2 : 0);
  memcpy(r.data, f.data, 8);
  if (++rawHead >= RAW_SIZE) rawHead = 0;
  if (rawCount < RAW_SIZE) rawCount++;
}

void addLog(const Entry &e, bool isNew) {
  LogLine &l = logBuf[logHead];
  l.t = e.lastChange;
  l.id = e.id;
  l.ext = e.ext;
  l.isNew = isNew;
  l.len = e.len;
  l.prevLen = isNew ? 0 : e.prevLen;
  memcpy(l.data, e.data, 8);
  memcpy(l.prev, e.prev, 8);
  if (++logHead >= LOG_SIZE) logHead = 0;
  if (logCount < LOG_SIZE) logCount++;
}

void handleFrame(const CANMessage &f) {
  const uint64_t now = nowUs();
  frameTotal++;
  health.bits += frameBits(f.ext, f.len);
  addRaw(f, now);
  if (f.rtr) health.rtrFrames++;

  Entry *hit = nullptr, *spare = nullptr;
  for (int i = 0; i < MAX_IDS; i++) {
    Entry &e = table_[i];
    if (!e.used) { if (!spare) spare = &e; continue; }
    if (e.id == f.id && e.ext == f.ext) { hit = &e; break; }
  }

  if (hit) {
    Entry &e = *hit;
    const uint64_t dt = now - e.lastSeen;
    frameStatsPeriod(e.st, dt);
    e.lastSeen = now;
    e.count++;
    if (f.rtr) { e.rtr = true; return; }           // remote frames carry no payload
    if (e.len != f.len || memcmp(e.data, f.data, f.len) != 0) {
      const uint32_t cdt = (uint32_t)((now - e.lastChange) / 1000ULL);
      frameStatsChange(e.st, f.data, e.data, f.len, cdt, e.changes != 0);
      memcpy(e.prev, e.data, 8);
      e.prevLen = e.len;
      e.len = f.len;
      memset(e.data, 0, 8);
      memcpy(e.data, f.data, f.len);
      e.changes++;
      e.lastChange = now;
      addLog(e, false);
    }
    return;
  }

  if (!spare) { health.droppedIds++; return; }     // table full
  Entry &e = *spare;
  memset(&e, 0, sizeof(e));
  e.used = true;
  e.id = f.id;
  e.ext = f.ext;
  e.rtr = f.rtr;
  e.len = f.len;
  memcpy(e.data, f.data, 8);
  frameStatsInit(e.st, f.data, f.len);
  e.count = 1;
  e.firstSeen = e.lastSeen = e.lastChange = now;
  addLog(e, true);
}

// ---------- Transmit (gated) ----------
static bool txAllowed = false;
static bool canOk = false;      // driver answered on SPI and entered a mode

void canISR() { can.isr(); }

// ACAN2515 refuses an ISR without an INT pin (kISRNotNullAndNoIntPin), so only hand
// it one when MCP2515_INT is actually wired. A plain variable, not a function
// returning a function pointer: the IDE's prototype generator cannot parse those.
static void (*const CAN_ISR)() = MCP2515_INT == 255 ? nullptr : canISR;

bool sendFrame(uint32_t id, bool ext, const uint8_t *d, uint8_t len) {
  if (!txAllowed || listenOnly) return false;
  CANMessage m;
  m.id = id;
  m.ext = ext;
  m.rtr = false;
  m.len = len;
  memset(m.data, 0xAA, 8);
  memcpy(m.data, d, len);
  for (uint8_t i = 0; i < 12; i++) {
    if (can.tryToSend(m)) return true;
    can.poll();
    delay(1);
  }
  return false;
}

// ---------- ISO-TP (ISO 15765-2) receive ----------
struct IsoRx {
  bool active, done;
  uint32_t reqId, respId;
  uint16_t total, len;
  uint8_t next, buf[64];
};
static IsoRx iso;

void isoTpFeed(const CANMessage &f) {
  if (!iso.active || f.rtr || f.ext || f.len < 1) return;
  const bool match = (f.id == iso.reqId) || (f.id == iso.reqId + 8) ||
                     (iso.reqId == 0x7DF && f.id >= 0x7E8 && f.id <= 0x7EF);
  if (!match) return;
  iso.respId = f.id;
  const uint8_t pci = f.data[0] >> 4;
  if (pci == 0) {                                   // single frame
    iso.total = f.data[0] & 0x0F;
    if (iso.total > sizeof(iso.buf)) iso.total = sizeof(iso.buf);
    if (iso.total > f.len - 1) iso.total = f.len - 1;
    iso.len = iso.total;
    memcpy(iso.buf, f.data + 1, iso.len);
    iso.done = true;
  } else if (pci == 1) {                            // first frame of a multi-frame reply
    iso.total = (uint16_t)(((f.data[0] & 0x0F) << 8) | f.data[1]);
    if (iso.total > sizeof(iso.buf)) iso.total = sizeof(iso.buf);
    iso.len = f.len > 2 ? (uint8_t)(f.len - 2) : 0;
    if (iso.len > iso.total) iso.len = iso.total;
    memcpy(iso.buf, f.data + 2, iso.len);
    iso.next = 1;
    const uint8_t fc[8] = {0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};   // CTS, block 0, STmin 0
    sendFrame(f.id, false, fc, 8);
  } else if (pci == 2) {                            // consecutive frame
    if (!iso.total || (f.data[0] & 0x0F) != iso.next) return;
    iso.next = (iso.next + 1) & 0x0F;
    uint8_t n = f.len - 1;
    if ((uint16_t)(iso.len + n) > iso.total) n = (uint8_t)(iso.total - iso.len);
    memcpy(iso.buf + iso.len, f.data + 1, n);
    iso.len += n;
    if (iso.len >= iso.total) iso.done = true;
  }
}

// Drain the controller once: keeps the sniffer live while a request is in flight.
void pumpOnce() {
  can.poll();
  CANMessage f;
  while (can.receive(f)) {
    isoTpFeed(f);
    handleFrame(f);
  }
}

bool isoTpRequest(uint32_t txId, const uint8_t *payload, uint8_t len, uint32_t timeoutMs) {
  if (!txAllowed || listenOnly || len > 7) return false;
  memset(&iso, 0, sizeof(iso));
  iso.reqId = txId;
  iso.active = true;
  uint8_t sf[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  sf[0] = (uint8_t)(len & 0x0F);
  memcpy(sf + 1, payload, len);
  if (!sendFrame(txId, false, sf, 8)) { iso.active = false; return false; }
  const uint64_t t0 = nowUs();
  while (!iso.done && (nowUs() - t0) < (uint64_t)timeoutMs * 1000ULL) {
    pumpOnce();
    delay(1);
  }
  iso.active = false;
  return iso.done;
}

// ---------- Driver ----------
// ponytail: every begin() leaks one ESP32 helper task (~5 KB). ACAN2515 has no
// teardown for it, so bitrate switching is the only place that pays; fine for the
// handful of switches a sniffer sees. Fix upstream or cache per-bitrate drivers.
// Copy the timing the library is about to program into health, including the
// CNF registers, so the UI can show what the MCP2515 actually got.
void applyTiming(const ACAN2515Settings &s) {
  health.abr = s.actualBitRate();
  const uint32_t ppm = s.ppmFromDesiredBitRate();
  health.ppm = (uint16_t)(ppm > 65535 ? 65535 : ppm);
  const uint32_t sp = s.samplePointFromBitStart();
  health.sp = (uint16_t)(sp > 65535 ? 65535 : sp);
  health.brp = s.mBitRatePrescaler;
  health.tq = (uint8_t)(1 + s.mPropagationSegment + s.mPhaseSegment1 + s.mPhaseSegment2);
  health.sjw = s.mSJW;
  health.cnf1 = (uint8_t)(((s.mSJW - 1) << 6) | ((s.mBitRatePrescaler - 1) & 0x3F));
  health.cnf2 = (uint8_t)(0x80 | (s.mTripleSampling ? 0x40 : 0) |
                          ((s.mPhaseSegment1 - 1) << 3) | ((s.mPropagationSegment - 1) & 0x07));
  health.cnf3 = (uint8_t)((s.mPhaseSegment2 - 1) & 0x07);
}

uint16_t applyCan() {
  can.end();
  ACAN2515Settings s(QUARTZ_FREQUENCY, bitrate);
  s.mRequestedMode = listenOnly ? ACAN2515Settings::ListenOnlyMode
                                : ACAN2515Settings::NormalMode;
  s.mReceiveBufferSize = 128;      // deeper software RX buffer: fewer frames lost during HTTP work
  applyTiming(s);
  const uint16_t err = can.begin(s, CAN_ISR);
  canOk = (err == 0);
  return err;
}

void appendId(Out &o, uint32_t id, bool ext) {
  char t[12];
  uint8_t n = 0;
  if (!id) t[n++] = '0';
  while (id) { t[n++] = "0123456789ABCDEF"[id & 15]; id >>= 4; }
  while (n) o.ch(t[--n]);
  if (ext) o.ch('x');
}

uint32_t busLoadPerMille() {
  const uint64_t us = nowUs() - healthBaseUs;
  if (!us || !bitrate) return 0;
  const uint64_t bps = health.bits * 1000000ULL / us;
  const uint64_t pm = bps * 1000ULL / bitrate;
  return (uint32_t)(pm > 65535 ? 65535 : pm);
}

void sampleHealth() {
  if (!canOk) {
    health.rec = health.tec = health.eflg = 0;
    health.rxPeak = health.rxSize = 0;
    return;
  }
  health.rec = can.receiveErrorCounter();
  health.tec = can.transmitErrorCounter();
  health.eflg = can.errorFlagRegister();
  health.eflgSeen |= health.eflg;
  health.rxPeak = can.receiveBufferPeakCount();
  health.rxSize = can.receiveBufferSize();
}

// ESP32 + MCP2515 headroom. Sent with every live poll: what is left on the
// board and what the controller is actually programmed with.
void appendSys(Out &o) {
  uint16_t used = 0;
  for (int i = 0; i < MAX_IDS; i++) if (table_[i].used) used++;
  // network: station or access point, whichever is up
  const wifi_mode_t wm = WiFi.getMode();
  const bool apNow = (wm == WIFI_AP || wm == WIFI_AP_STA);
  const int clients = apNow ? WiFi.softAPgetStationNum() : 0;
  uint8_t chan = 0;
  wifi_second_chan_t second;
  if (wm != WIFI_OFF) esp_wifi_get_channel(&chan, &second);
  const String ssidStr = apNow ? WiFi.softAPSSID() : WiFi.SSID();
  char ssid[33];
  uint8_t sn = 0;
  for (uint16_t i = 0; i < ssidStr.length() && sn < sizeof(ssid) - 1; i++) {
    const char c = ssidStr[i];
    ssid[sn++] = (c == '"' || c == '\\' || (uint8_t)c < 0x20) ? '_' : c;   // keep the JSON valid
  }
  ssid[sn] = 0;
  const IPAddress ip = apNow ? WiFi.softAPIP() : WiFi.localIP();
  char ips[16];
  snprintf(ips, sizeof(ips), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  const long rssi = (!apNow && WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  o.raw("\"sys\":{");
  o.raw("\"heap\":"); o.num((uint32_t)ESP.getFreeHeap());
  o.raw(",\"hmin\":"); o.num((uint32_t)ESP.getMinFreeHeap());
  o.raw(",\"blk\":"); o.num((uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  o.raw(",\"stack\":"); o.num((uint32_t)uxTaskGetStackHighWaterMark(NULL));
  o.raw(",\"cpu\":"); o.num((uint32_t)getCpuFrequencyMhz());
  o.raw(",\"cores\":"); o.num((uint32_t)ESP.getChipCores());
  o.raw(",\"flash\":"); o.num((uint32_t)(ESP.getFlashChipSize() >> 10));
  o.raw(",\"sketch\":"); o.num((uint32_t)(ESP.getSketchSize() >> 10));
  o.raw(",\"chip\":\""); o.raw(ESP.getChipModel()); o.ch('"');
  o.raw(",\"ip\":\""); o.raw(ips); o.ch('"');
  o.raw(",\"rssi\":");
  if (rssi < 0) { o.ch('-'); o.num((uint64_t)(-rssi)); } else o.num((uint64_t)rssi);
  o.raw(",\"ap\":"); o.num(apNow ? 1 : 0);
  o.raw(",\"apcli\":"); o.num((uint32_t)clients);
  o.raw(",\"apsec\":"); o.num((apNow && strlen(AP_PASSWORD) >= 8) ? 1 : 0);
  o.raw(",\"chan\":"); o.num((uint32_t)chan);
  o.raw(",\"ssid\":\""); o.raw(ssid); o.ch('"');
  o.raw(",\"ids\":"); o.num(used);
  o.raw(",\"idsz\":"); o.num((uint32_t)MAX_IDS);
  o.raw(",\"raw\":"); o.num((uint32_t)rawCount);
  o.raw(",\"rawsz\":"); o.num((uint32_t)RAW_SIZE);
  o.raw(",\"quartz\":"); o.num((uint32_t)(QUARTZ_FREQUENCY / 1000));
  o.raw(",\"spi\":"); o.num(10000);          // ACAN2515 uses 10 MHz on the SPI bus
  o.raw(",\"brp\":"); o.num(health.brp);
  o.raw(",\"tq\":"); o.num(health.tq);
  o.raw(",\"sjw\":"); o.num(health.sjw);
  o.raw(",\"cnf1\":"); o.num(health.cnf1);
  o.raw(",\"cnf2\":"); o.num(health.cnf2);
  o.raw(",\"cnf3\":"); o.num(health.cnf3);
  o.ch('}');
}

// /data            live poll payload (ids + change log)
// /data?full=1&from=N&count=M   page through the table with every per-ID statistic,
//                                no log: used to capture a sample for comparison
void sendData() {
  const bool full = server.hasArg("full");
  int skip = full ? server.arg("from").toInt() : 0;
  int count = full ? server.arg("count").toInt() : 0;
  if (count <= 0) count = 32;
  if (count > 64) count = 64;
  if (skip < 0) skip = 0;
  sampleHealth();
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.ch('{');
  o.key("t"); o.num(nowMs());
  o.raw(",\"n\":"); o.num(frameTotal);
  o.raw(",\"b\":"); o.num(bitrate);
  o.raw(",\"lo\":"); o.num(listenOnly ? 1 : 0);
  o.raw(",\"tx\":"); o.num(txAllowed ? 1 : 0);
  o.raw(",\"atx\":"); o.num(ALLOW_TX ? 1 : 0);
  o.raw(",\"h\":{");
  o.raw("\"canok\":"); o.num(canOk ? 1 : 0);
  o.raw(",\"load\":"); o.num(busLoadPerMille());
  o.raw(",\"rec\":"); o.num(health.rec);
  o.raw(",\"tec\":"); o.num(health.tec);
  o.raw(",\"eflg\":"); o.num(health.eflg);
  o.raw(",\"efl\":"); o.num(health.eflgSeen);
  o.raw(",\"abr\":"); o.num(health.abr);
  o.raw(",\"ppm\":"); o.num(health.ppm);
  o.raw(",\"sp\":"); o.num(health.sp);
  o.raw(",\"peak\":"); o.num(health.rxPeak);
  o.raw(",\"rxsz\":"); o.num(health.rxSize);
  o.raw(",\"drop\":"); o.num(health.droppedIds);
  o.raw(",\"rtr\":"); o.num(health.rtrFrames);
  o.ch('}');
  if (!full) { o.ch(','); appendSys(o); }
  o.raw(",\"ids\":[");
  bool first = true;
  bool more = false;
  int emitted = 0;
  for (int i = 0; i < MAX_IDS; i++) {
    const Entry &e = table_[i];
    if (!e.used) continue;
    if (skip > 0) { skip--; continue; }
    if (emitted >= count) { more = true; break; }
    emitted++;
    if (!first) o.ch(',');
    first = false;
    o.raw("{\"id\":\""); appendId(o, e.id, e.ext);
    o.raw("\",\"l\":"); o.num(e.len);
    o.raw(",\"d\":\""); o.hex(e.data, e.len);
    o.raw("\",\"p\":\""); if (e.changes) o.hex(e.prev, e.prevLen);
    o.raw("\",\"n\":"); o.num(e.count);
    o.raw(",\"c\":"); o.num(e.changes);
    o.raw(",\"lc\":"); o.num(e.lastChange / 1000ULL);
    o.raw(",\"fs\":"); o.num(e.firstSeen / 1000ULL);
    o.raw(",\"ls\":"); o.num(e.lastSeen / 1000ULL);
    o.raw(",\"pm\":"); o.num(e.st.pMin == 0xFFFFFFFF ? 0 : e.st.pMin);
    o.raw(",\"pa\":"); o.num(e.st.pN ? (e.st.pSum / e.st.pN) : 0ULL);
    o.raw(",\"px\":"); o.num(e.st.pMax);
    if (full) {                                  // extra fields only a capture needs
      o.raw(",\"r\":"); o.num(e.rtr ? 1 : 0);
      o.raw(",\"o\":\""); o.hex(e.st.orB, 8); o.ch('"');
      o.raw(",\"a\":\""); o.hex(e.st.andB, 8); o.ch('"');
      o.raw(",\"t\":\""); o.hex(e.st.togMask, 8); o.ch('"');
      o.raw(",\"bc\":[");
      for (uint8_t b = 0; b < 8; b++) { if (b) o.ch(','); o.num(e.st.nChg[b]); }
      o.raw("],\"cu\":[");
      for (uint8_t b = 0; b < 8; b++) { if (b) o.ch(','); o.num(e.st.cntUp[b]); }
      o.ch(']');
      o.raw(",\"cm\":"); o.num(e.st.cMin == 0xFFFFFFFF ? 0 : e.st.cMin);
      o.raw(",\"cx\":"); o.num(e.st.cMax);
    }
    o.ch('}');
  }
  if (full) { o.raw("],\"more\":"); o.num(more ? 1 : 0); o.raw(",\"log\":[]"); }
  else {
    o.raw("],\"log\":[");
    first = true;
    for (int k = 0; k < logCount; k++) {
      const int idx = (logHead - 1 - k + LOG_SIZE) % LOG_SIZE;   // newest first
      const LogLine &l = logBuf[idx];
      if (!first) o.ch(',');
      first = false;
      o.raw("{\"t\":"); o.num(l.t / 1000ULL);
      o.raw(",\"id\":\""); appendId(o, l.id, l.ext);
      o.raw("\",\"d\":\""); o.hex(l.data, l.len);
      o.raw("\",\"p\":\""); if (!l.isNew) o.hex(l.prev, l.prevLen);
      o.raw("\"}");
    }
    o.raw("]");
  }
  o.raw("}");
  o.end();
  server.send(200, "application/json", outBuf);
}

bool parseBusId(const String &s, uint32_t &id, bool &ext) {
  ext = false;
  String t = s;
  t.trim();
  if (t.endsWith("x") || t.endsWith("X")) { ext = true; t.remove(t.length() - 1); }
  if (!t.length()) return false;
  char *endp = nullptr;
  const unsigned long v = strtoul(t.c_str(), &endp, 16);
  if (endp && *endp) return false;
  id = (uint32_t)v;
  return true;
}

Entry *findEntry(uint32_t id, bool ext) {
  for (int i = 0; i < MAX_IDS; i++)
    if (table_[i].used && table_[i].id == id && table_[i].ext == ext) return &table_[i];
  return nullptr;
}

void handleDetail() {
  uint32_t id = 0;
  bool ext = false;
  Out o = {outBuf, OUT_LIMIT, 0, false};
  if (!parseBusId(server.arg("id"), id, ext)) { server.send(400, "application/json", "{\"e\":1}"); return; }
  Entry *p = findEntry(id, ext);
  if (!p) { server.send(404, "application/json", "{\"e\":2}"); return; }
  const Entry &e = *p;
  o.ch('{');
  o.key("id"); o.ch('"'); appendId(o, e.id, e.ext); o.ch('"');
  o.raw(",\"l\":"); o.num(e.len);
  o.raw(",\"rtr\":"); o.num(e.rtr ? 1 : 0);
  o.raw(",\"d\":\""); o.hex(e.data, e.len); o.ch('"');
  o.raw(",\"o\":\""); o.hex(e.st.orB, 8); o.ch('"');
  o.raw(",\"a\":\""); o.hex(e.st.andB, 8); o.ch('"');
  o.raw(",\"t\":\""); o.hex(e.st.togMask, 8); o.ch('"');
  o.raw(",\"bc\":[");
  for (uint8_t i = 0; i < 8; i++) { if (i) o.ch(','); o.num(e.st.nChg[i]); }
  o.raw("],\"cu\":[");
  for (uint8_t i = 0; i < 8; i++) { if (i) o.ch(','); o.num(e.st.cntUp[i]); }
  o.ch(']');
  o.raw(",\"n\":"); o.num(e.count);
  o.raw(",\"c\":"); o.num(e.changes);
  o.raw(",\"lc\":"); o.num(e.lastChange / 1000ULL);
  o.raw(",\"fs\":"); o.num(e.firstSeen / 1000ULL);
  o.raw(",\"ls\":"); o.num(e.lastSeen / 1000ULL);
  o.raw(",\"pm\":"); o.num(e.st.pMin == 0xFFFFFFFF ? 0 : e.st.pMin);
  o.raw(",\"pa\":"); o.num(e.st.pN ? (e.st.pSum / e.st.pN) : 0ULL);
  o.raw(",\"px\":"); o.num(e.st.pMax);
  o.raw(",\"cm\":"); o.num(e.st.cMin == 0xFFFFFFFF ? 0 : e.st.cMin);
  o.raw(",\"cx\":"); o.num(e.st.cMax);
  o.ch('}');
  o.end();
  server.send(200, "application/json", outBuf);
}

// ---------- Exports ----------
void sendAttachment(const char *name, const char *type) {
  server.sendHeader("Content-Disposition", String("attachment; filename=\"") + name + "\"");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, type, outBuf);
}

void exportCsv() {
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("id,ext,rtr,len,data,count,changes,first_seen_ms,last_seen_ms,"
        "period_min_us,period_avg_us,period_max_us,change_min_ms,change_max_ms,"
        "or_mask,and_mask,toggled_mask,byte_changes,inc16_hits\n");
  for (int i = 0; i < MAX_IDS; i++) {
    const Entry &e = table_[i];
    if (!e.used) continue;
    appendId(o, e.id, false);
    o.raw(e.ext ? ",1" : ",0");
    o.raw(e.rtr ? ",1," : ",0,");
    o.num(e.len); o.ch(',');
    o.hex(e.data, e.len);
    o.ch(','); o.num(e.count);
    o.ch(','); o.num(e.changes);
    o.ch(','); o.num(e.firstSeen / 1000ULL);
    o.ch(','); o.num(e.lastSeen / 1000ULL);
    o.ch(','); o.num(e.st.pMin == 0xFFFFFFFF ? 0 : e.st.pMin);
    o.ch(','); o.num(e.st.pN ? (e.st.pSum / e.st.pN) : 0ULL);
    o.ch(','); o.num(e.st.pMax);
    o.ch(','); o.num(e.st.cMin == 0xFFFFFFFF ? 0 : e.st.cMin);
    o.ch(','); o.num(e.st.cMax);
    o.ch(','); o.hex(e.st.orB, 8);
    o.ch(','); o.hex(e.st.andB, 8);
    o.ch(','); o.hex(e.st.togMask, 8);
    o.ch('"'); for (uint8_t b = 0; b < 8; b++) { if (b) o.ch('|'); o.num(e.st.nChg[b]); } o.ch('"');
    o.ch('"'); for (uint8_t b = 0; b < 8; b++) { if (b) o.ch('|'); o.num(e.st.cntUp[b]); } o.ch('"');
    o.ch('\n');
  }
  o.end();
  sendAttachment("canwatch-ids.csv", "text/csv; charset=utf-8");
}

void exportRawCsv() {
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("seq,time_us,id,ext,rtr,len,data\n");
  for (int k = 0; k < rawCount; k++) {
    const int idx = (rawHead - rawCount + k + RAW_SIZE) % RAW_SIZE;
    const Raw &r = rawBuf[idx];
    o.num((uint64_t)k); o.ch(',');
    o.num(r.tUs); o.ch(',');
    appendId(o, r.id, false); o.ch(',');
    o.num(r.flags & 1 ? 1 : 0); o.ch(',');
    o.num(r.flags & 2 ? 1 : 0); o.ch(',');
    o.num(r.len); o.ch(',');
    o.hex(r.data, r.len);
    o.ch('\n');
  }
  o.end();
  sendAttachment("canwatch-raw.csv", "text/csv; charset=utf-8");
}

void exportCandump() {
  Out o = {outBuf, OUT_LIMIT, 0, false};
  for (int k = 0; k < rawCount; k++) {
    const int idx = (rawHead - rawCount + k + RAW_SIZE) % RAW_SIZE;
    const Raw &r = rawBuf[idx];
    o.ch('(');
    o.num(r.tUs / 1000000ULL);
    o.ch('.');
    const uint32_t frac = (uint32_t)(r.tUs % 1000000ULL);
    char f[8];
    snprintf(f, sizeof(f), "%06lu", (unsigned long)frac);
    o.raw(f);
    o.raw(") can0 ");
    appendId(o, r.id, false);
    if (r.flags & 1) o.ch('x');
    o.ch('#'); o.hex(r.data, r.len);
    o.ch('\n');
  }
  o.end();
  sendAttachment("canwatch.log", "text/plain; charset=utf-8");
}

// ---------- OBD-II ----------
struct Pid {
  uint8_t id;
  const char *name;
  const char *unit;
  float mulA, mulB, add;
};
static const Pid PIDS[] = {
  {0x04, "Engine load",   "%",    100.0f / 255, 0,            0},
  {0x05, "Coolant",       "C",    1,            0,            -40},
  {0x06, "STFT bank 1",   "%",    100.0f / 128, 0,            -100},
  {0x07, "LTFT bank 1",   "%",    100.0f / 128, 0,            -100},
  {0x0A, "Fuel pressure", "kPa",  3,            0,            0},
  {0x0B, "Intake MAP",    "kPa",  1,            0,            0},
  {0x0C, "RPM",           "rpm",  1.0f / 4,     256.0f / 4,   0},
  {0x0D, "Speed",         "km/h", 1,            0,            0},
  {0x0E, "Timing advance","deg",  1.0f / 2,     0,            -64},
  {0x0F, "Intake air",    "C",    1,            0,            -40},
  {0x10, "MAF",           "g/s",  1.0f / 100,   256.0f / 100, 0},
  {0x11, "Throttle",      "%",    100.0f / 255, 0,            0},
  {0x1F, "Run time",      "s",    1,            256,          0},
  {0x21, "Since MIL",     "km",   1,            256,          0},
  {0x2F, "Fuel level",    "%",    100.0f / 255, 0,            0},
  {0x33, "Barometric",    "kPa",  1,            0,            0},
  {0x42, "Module voltage","V",    1.0f / 1000,  256.0f / 1000, 0},
  {0x5C, "Oil temp",      "C",    1,            0,            -40},
};
static const uint8_t PID_COUNT = sizeof(PIDS) / sizeof(PIDS[0]);
static uint32_t supMask[3] = {0, 0, 0};

bool obd01(uint8_t pid) {
  uint8_t req[2] = {0x01, pid};
  if (!isoTpRequest(0x7DF, req, 2, 150)) return false;
  return iso.len >= 3 && iso.buf[0] == 0x41 && iso.buf[1] == pid;
}

bool pidSupported(uint8_t pid) {
  const uint8_t k = (uint8_t)((pid - 1) / 0x20);
  const uint8_t b = (uint8_t)((pid - 1) % 0x20);
  if (k > 2) return false;
  return (supMask[k] >> (31 - b)) & 1;
}

void scanSupportedPids() {
  supMask[0] = supMask[1] = supMask[2] = 0;
  for (uint8_t k = 0; k < 3; k++) {
    uint8_t req[2] = {0x01, (uint8_t)(k * 0x20)};
    if (isoTpRequest(0x7DF, req, 2, 150) && iso.len >= 6 &&
        iso.buf[0] == 0x41 && iso.buf[1] == k * 0x20) {
      supMask[k] = ((uint32_t)iso.buf[2] << 24) | ((uint32_t)iso.buf[3] << 16) |
                   ((uint32_t)iso.buf[4] << 8) | iso.buf[5];
    }
  }
}

void txFail(const char *msg) {
  server.send(200, "application/json", String("{\"ok\":0,\"e\":\"") + msg + "\"}");
}

bool txReady() {
  if (!ALLOW_TX) { txFail("transmit compiled out"); return false; }
  if (!canOk) { txFail("CAN controller is down"); return false; }
  if (listenOnly) { txFail("listen-only mode"); return false; }
  if (!txAllowed) { txFail("transmit not enabled"); return false; }
  return true;
}

void handleReq() {
  if (!txReady()) return;
  uint32_t id = 0;
  bool ext = false;
  if (!parseBusId(server.arg("id"), id, ext)) { txFail("bad id"); return; }
  if (!id || id > 0x7FF) { txFail("standard id 1..0x7FF"); return; }
  const String ds = server.arg("data");
  if (ds.length() % 2) { txFail("payload hex must have an even number of digits"); return; }
  if (ds.length() > 14) { txFail("single frame holds 7 bytes; no multi-frame transmit"); return; }
  uint8_t payload[7];
  uint8_t n = 0;
  for (unsigned int i = 0; i + 1 < ds.length() && n < 7; i += 2) {
    char b[3] = {ds[i], ds[i + 1], 0};
    payload[n++] = (uint8_t)strtoul(b, nullptr, 16);
  }
  if (!n) { txFail("empty payload"); return; }
  if (ext) { txFail("extended ids not supported for requests"); return; }
  Out o = {outBuf, OUT_LIMIT, 0, false};
  if (!isoTpRequest(id, payload, n, 400)) { txFail("no response"); return; }
  o.raw("{\"ok\":1,\"src\":\""); appendId(o, iso.respId, false);
  o.raw("\",\"d\":\""); o.hex(iso.buf, (uint8_t)iso.len); o.raw("\"}");
  o.end();
  server.send(200, "application/json", outBuf);
}

void handleObd() {
  if (!txReady()) return;
  Out o = {outBuf, OUT_LIMIT, 0, false};
  if (server.hasArg("pid")) {
    const uint8_t pid = (uint8_t)strtoul(server.arg("pid").c_str(), nullptr, 16);
    if (!obd01(pid)) { txFail("no response"); return; }
    o.raw("{\"ok\":1,\"pid\":\""); o.hex(&pid, 1);
    o.raw("\",\"d\":\""); o.hex(iso.buf, (uint8_t)iso.len); o.ch('"');
    o.raw(",\"v\":[");
    for (uint16_t i = 2; i < iso.len; i++) { if (i > 2) o.ch(','); o.num(iso.buf[i]); }
    o.raw("]}");
    o.end();
    server.send(200, "application/json", outBuf);
    return;
  }
  scanSupportedPids();
  o.raw("{\"ok\":1,\"sup\":[");
  for (uint8_t k = 0; k < 3; k++) { if (k) o.ch(','); o.num(supMask[k]); }
  o.raw("],\"pids\":[");
  bool first = true;
  for (uint8_t i = 0; i < PID_COUNT; i++) {
    const Pid &p = PIDS[i];
    if (!pidSupported(p.id)) continue;
    if (!obd01(p.id)) continue;
    const float a = iso.buf[2];
    const float b = iso.len > 3 ? iso.buf[3] : 0;
    if (!first) o.ch(',');
    first = false;
    o.raw("{\"p\":\""); o.hex(&p.id, 1);
    o.raw("\",\"n\":\""); o.raw(p.name);
    o.raw("\",\"u\":\""); o.raw(p.unit);
    o.raw("\",\"v\":"); o.f2(a * p.mulA + b * p.mulB + p.add);
    o.ch('}');
  }
  o.raw("]}");
  o.end();
  server.send(200, "application/json", outBuf);
}

void handleVin() {
  if (!txReady()) return;
  const uint8_t req[3] = {0x02, 0x09, 0x02};
  if (!isoTpRequest(0x7DF, req, 3, 500) || iso.len < 5 || iso.buf[0] != 0x49) {
    txFail("no VIN response");
    return;
  }
  char vin[24];
  uint8_t n = 0;
  for (uint16_t i = 3; i < iso.len && n < sizeof(vin) - 1; i++) {
    const char c = (char)iso.buf[i];
    if (c >= 0x20 && c < 0x7F) vin[n++] = c;
  }
  vin[n] = 0;
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("{\"ok\":1,\"vin\":\""); o.raw(vin); o.raw("\"}");
  o.end();
  server.send(200, "application/json", outBuf);
}

void handleDtc() {
  if (!txReady()) return;
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("{\"ok\":1");
  uint8_t req1[2] = {0x01, 0x01};
  if (isoTpRequest(0x7DF, req1, 2, 200) && iso.len >= 3 && iso.buf[0] == 0x41 && iso.buf[1] == 0x01) {
    o.raw(",\"mil\":"); o.num((iso.buf[2] & 0x80) ? 1 : 0);
    o.raw(",\"count\":"); o.num(iso.buf[2] & 0x7F);
  }
  const uint8_t req3[1] = {0x03};
  o.raw(",\"codes\":[");
  bool first = true;
  if (isoTpRequest(0x7DF, req3, 1, 400) && iso.len >= 2 && iso.buf[0] == 0x43) {
    for (uint16_t i = 2; i + 1 < iso.len; i += 2) {
      const uint8_t a = iso.buf[i], b = iso.buf[i + 1];
      if (!a && !b) continue;
      char code[8];
      snprintf(code, sizeof(code), "%c%u%X%X%X", "PCBU"[(a >> 6) & 3],
               (a >> 4) & 3, a & 0x0F, (b >> 4) & 0x0F, b & 0x0F);
      if (!first) o.ch(',');
      first = false;
      o.ch('"'); o.raw(code); o.ch('"');
    }
  }
  o.raw("]}");
  o.end();
  server.send(200, "application/json", outBuf);
}

// ---------- Control ----------
void clearAll() {
  memset(table_, 0, sizeof(table_));
  logHead = 0; logCount = 0;
  rawHead = 0; rawCount = 0;
  frameTotal = 0;
  health.bits = 0;
  health.droppedIds = 0;
  health.rtrFrames = 0;
  health.eflgSeen = 0;
  healthBaseUs = nowUs();
  server.send(200, "text/plain", "ok");
}

void handleMode() {
  const bool want = server.arg("listen").toInt() != 0;
  uint16_t err = 0;
  if (want != listenOnly) {
    err = can.changeModeOnTheFly(want ? ACAN2515Settings::ListenOnlyMode
                                      : ACAN2515Settings::NormalMode);
    if (err == 0) {
      listenOnly = want;
      if (listenOnly) txAllowed = false;     // never transmit from a passive bus
    }
    Serial.print("CAN mode -> "); Serial.println(listenOnly ? "LISTEN ONLY" : "NORMAL (ACKs frames)");
  }
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("{\"lo\":"); o.num(listenOnly ? 1 : 0);
  o.raw(",\"err\":"); o.num(err);
  o.raw(",\"tx\":"); o.num(txAllowed ? 1 : 0); o.ch('}');
  o.end();
  server.send(200, "application/json", outBuf);
}

void handleBitrate() {
  const uint32_t hz = (uint32_t)server.arg("hz").toInt();
  bool known = false;
  for (uint8_t i = 0; i < BITRATE_COUNT; i++) if (BITRATES[i] == hz) known = true;
  uint16_t err = 0;
  if (!known) {
    err = 1;                    // unsupported value, keep current bitrate
  } else if (hz != bitrate) {
    const uint32_t old = bitrate;
    bitrate = hz;
    err = applyCan();
    if (err) {                  // roll back so the bus keeps working
      bitrate = old;
      applyCan();
    } else {                    // frames on other bitrates are not comparable
      memset(table_, 0, sizeof(table_));
      logHead = 0; logCount = 0; frameTotal = 0;
      rawHead = 0; rawCount = 0;
      health.bits = 0;
      health.droppedIds = 0;
      health.rtrFrames = 0;
      health.eflgSeen = 0;
      healthBaseUs = nowUs();
      txAllowed = false;
    }
    Serial.print("CAN bitrate -> "); Serial.println(bitrate);
  }
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("{\"b\":"); o.num(bitrate);
  o.raw(",\"err\":"); o.num(err); o.ch('}');
  o.end();
  server.send(200, "application/json", outBuf);
}

void handleTx() {
  const bool en = server.arg("en").toInt() != 0;
  uint8_t err = 0;
  if (en) {
    if (!ALLOW_TX) err = 1;          // compiled out
    else if (listenOnly) err = 2;    // passive bus, no ACK, nothing to talk to
    else txAllowed = true;
  } else {
    txAllowed = false;
  }
  Out o = {outBuf, OUT_LIMIT, 0, false};
  o.raw("{\"tx\":"); o.num(txAllowed ? 1 : 0);
  o.raw(",\"err\":"); o.num(err); o.ch('}');
  o.end();
  server.send(200, "application/json", outBuf);
}

// ---------- Web page ----------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#161616">
<title>Canwatch</title>
<style>
:root{color-scheme:dark;
  /* opencode console tokens: grey ramp + alpha overlays (dark scheme) */
  --bg:#161616;--bg-deep:#080808;--surface:#242424;--layer2:#2e2e2e;--layer3:#3a3a3a;
  --fg:#fafafa;--muted:#aeaeae;--faint:#808080;
  --ring:#ffffff1f;--ring-muted:#ffffff14;--ring-strong:#fff3;
  --acc:#3b5cf6;--focus:#7698fd;
  --ok:#6bd586;--warn:#f2cf76;--bad:#f6a4a4;
  --ok-bg:#6bd5861f;--warn-bg:#f2cf7614;--bad-bg:#f6a4a41a;
  --elev-raised:0 2px 4px 0 #0000000a,0 1px 2px -1px #ffffff14,0 0 0 .5px #ffffff1f;
  --elev-elements:0 .5px .5px 0 #0006;
  --elev-btn:0 1px 1.5px 0 #ffffff1a;
  --sw-on:inset 0 2px 2px 0 #ffffff1a,inset 0 1px 1px 0 #ffffff1a,inset 0 0 0 .5px #fff3;
  --sw-off:inset 0 1px 1px 0 #ffffff14,inset 0 .5px .5px 0 #ffffff14,inset 0 0 0 .5px #ffffff1a;
  --g0:#7698fd;--g1:#6bd586;--g2:#f2cf76;--g3:#f6a4a4;--g4:#b9a6ff;--g5:#7fd3e0;--g6:#f0a8c0;--g7:#aeaeae;
  --h:28px;--h-sm:24px;--g:8px;--p:16px;--sp:24px;--gap:12px;
  --r:6px;--r-sm:4px;--r-lg:8px;
  --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;
  --sans:"Inter",sans-serif;
  --padx:max(var(--p),calc((100% - 1440px) / 2 + var(--p)))}
[data-theme="light"]{color-scheme:light;
  --bg:#fff;--bg-deep:#f2f2f2;--surface:#fafafa;--layer2:#f2f2f2;--layer3:#eee;
  --fg:#161616;--muted:#5c5c5c;--faint:gray;
  --ring:#0000001f;--ring-muted:#00000014;--ring-strong:#0003;
  --acc:#3b5cf6;--focus:#3b5cf6;
  --ok:#198b43;--warn:#8a6410;--bad:#a32f26;
  --ok-bg:#198b4314;--warn-bg:#cb9f3414;--bad-bg:#a32f2612;
  --elev-raised:0 2px 4px 0 #0000000a,0 1px 2px -1px #00000014,0 0 0 .5px #0000001f;
  --elev-elements:0 .5px .5px 0 #0006;
  --elev-btn:0 1px 1.5px 0 #0000001a;
  --sw-on:inset 0 2px 2px 0 #0000001a,inset 0 1px 1px 0 #0000001a,inset 0 0 0 .5px #0003;
  --sw-off:inset 0 1px 1px 0 #00000014,inset 0 .5px .5px 0 #00000014,inset 0 0 0 .5px #0000001a;
  --g0:#3b5cf6;--g1:#198b43;--g2:#8a6410;--g3:#a32f26;--g4:#5b3f8a;--g5:#0f6a72;--g6:#8e2f63;--g7:#4a5240}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
[hidden]{display:none!important}
html{-webkit-text-size-adjust:100%}
body{margin:0;background:var(--bg);color:var(--fg);font-family:var(--sans);font-size:13px;
  font-weight:440;line-height:20px;letter-spacing:-.04px;font-variant-numeric:tabular-nums;
  -webkit-font-smoothing:antialiased;text-rendering:geometricprecision;
  padding-bottom:calc(40px + env(safe-area-inset-bottom))}
/* ---- top bar ---- */
header{position:sticky;top:0;z-index:5;display:flex;flex-wrap:wrap;gap:var(--g);align-items:center;
  padding:calc(8px + env(safe-area-inset-top)) var(--padx) 8px;
  background:var(--bg);border-bottom:.5px solid var(--ring-muted)}
button,a.btn{position:relative;display:inline-flex;align-items:center;justify-content:center;gap:6px;
  height:var(--h);padding:0 11px;border:.5px solid var(--ring-muted);border-radius:var(--r);
  background-color:var(--surface);
  color:var(--fg);font-family:inherit;font-size:13px;font-weight:530;line-height:20px;
  letter-spacing:-.04px;white-space:nowrap;text-decoration:none;cursor:pointer;user-select:none;
  outline:none;box-shadow:var(--elev-btn);
  transition:background-color 85ms ease-out,color 85ms ease-out}
button:hover,a.btn:hover{background-image:linear-gradient(#ffffff0f,#ffffff0f)}
button:active,a.btn:active{background-image:linear-gradient(#ffffff14,#ffffff14)}
[data-theme="light"] button:hover,[data-theme="light"] a.btn:hover{background-image:linear-gradient(#0000000a,#0000000a)}
[data-theme="light"] button:active,[data-theme="light"] a.btn:active{background-image:linear-gradient(#00000014,#00000014)}
button:focus-visible,a.btn:focus-visible,select:focus-visible,input:focus-visible{
  outline:2px solid var(--focus);outline-offset:2px}
button:disabled{opacity:.45;cursor:not-allowed;box-shadow:none}
/* switch: 24x16 track, 12x12 thumb, 4px/2px radii, translate 8px (console spec) */
.toggles{display:flex;gap:var(--g);min-width:0;margin-left:-11px}
.tgl{justify-content:space-between;gap:10px;color:var(--fg);box-shadow:none;
  background-color:transparent;border-color:transparent}
.tgl:hover{background-image:linear-gradient(#ffffff0a,#ffffff0a)}
.tgl .track{position:relative;flex:0 0 auto;width:24px;height:16px;padding:2px;border-radius:var(--r-sm);
  background-color:var(--layer3);box-shadow:var(--sw-off);transition:background-color 85ms ease-out}
.tgl .knob{display:block;width:12px;height:12px;border-radius:2px;
  background-color:#aeaeae;box-shadow:var(--elev-elements);
  transition:transform 85ms ease-out,width 85ms ease-out,border-radius 85ms ease-out}
.tgl:hover .knob{width:13px;border-radius:3px}
.tgl[aria-checked="true"] .track{background-color:var(--acc);box-shadow:var(--sw-on)}
.tgl[aria-checked="true"] .knob{transform:translateX(8px);background-color:#eee}
.tgl[aria-checked="true"]:hover .knob{transform:translateX(7px)}
#mode[aria-checked="false"]{color:var(--bad)}
#mode[aria-checked="false"] .track{background-color:var(--bad);box-shadow:none;opacity:.75}
#mode[aria-checked="false"] .knob{transform:translateX(0)}
#fz.on{background-image:linear-gradient(#ffffff1a,#ffffff1a);color:var(--fg)}
.dot{width:6px;height:6px;border-radius:2px;background:var(--bad);flex:0 0 auto;transition:.2s}
.dot.ok{background:var(--ok)}
.dot.warn{background:var(--warn)}
header>*{flex:1 1 auto;min-width:0}
.toggles,.tgl,#bars,#theme,#br{flex:0 0 auto}
.grp{display:flex;gap:var(--g);align-items:center;min-width:0}
.grp.grow{flex:2 1 110px}
.actions{display:flex;gap:var(--g);flex:1 1 auto}
.actions button{flex:0 0 auto}
#bars{display:flex;gap:var(--g);align-items:center;flex-wrap:wrap}
/* tags */
.pill,.badge,.ext,.lost,.flag,.new{display:inline-flex;align-items:center;height:18px;padding:0 5px;
  border-radius:var(--r-sm);font-size:11px;font-weight:530;line-height:16px;letter-spacing:0;
  white-space:nowrap;vertical-align:1px}
.pill{height:var(--h);padding:0 9px;gap:6px;font-size:13px;border-radius:var(--r);
  border-color:var(--ring-muted);background-color:var(--surface);color:var(--muted);
  box-shadow:var(--elev-btn)}
.badge{margin-left:4px;justify-content:center;background-color:var(--acc);color:#fff}
.badge.bad{background-color:var(--bad);color:#2a0d0a}
.ext{margin-left:5px;background-color:var(--layer2);color:var(--faint);font-weight:440}
.lost{margin-left:5px;background-color:var(--bad-bg);color:var(--bad)}
.flag{margin:0;background-color:var(--bad-bg);color:var(--bad);border-color:#f6a4a44d}
.new{background-color:var(--ok);color:#04230f}
#q{flex:1 1 220px;min-width:120px;height:var(--h);padding:0 10px;border-radius:var(--r);
  border:.5px solid var(--ring-muted);background-color:var(--surface);color:var(--fg);
  font-family:inherit;font-size:13px;
  letter-spacing:-.04px;box-shadow:var(--elev-btn)}
#q::placeholder{color:var(--faint)}
#q:focus,#br:focus,#sortsel:focus,#theme:focus{outline:2px solid var(--focus);outline-offset:0}
select{appearance:none;-webkit-appearance:none;cursor:pointer;background-repeat:no-repeat;
  background-position:right 8px center;background-size:10px 6px;
  background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='10' height='6'%3E%3Cpath d='M1 1l4 4 4-4' fill='none' stroke='%23aeaeae' stroke-width='1.6' stroke-linecap='round' stroke-linejoin='round'/%3E%3C/svg%3E")}
[data-theme="light"] select{background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='10' height='6'%3E%3Cpath d='M1 1l4 4 4-4' fill='none' stroke='%235c5c5c' stroke-width='1.6' stroke-linecap='round' stroke-linejoin='round'/%3E%3C/svg%3E")}
#br,#theme{height:var(--h);padding:0 26px 0 11px;line-height:var(--h);
  border:.5px solid var(--ring-muted);border-radius:var(--r);
  background-color:var(--surface);color:var(--fg);font-family:inherit;font-size:13px;
  font-weight:530;letter-spacing:-.04px;box-shadow:var(--elev-btn)}
#sortsel{display:none;margin-left:auto;height:var(--h-sm);padding:0 22px 0 9px;line-height:var(--h-sm);
  border:.5px solid var(--ring-muted);border-radius:var(--r-sm);background-color:var(--surface);color:var(--muted);
  font-family:inherit;font-size:12px;box-shadow:var(--elev-btn)}
#warn{margin:0;padding:8px var(--padx);background:var(--warn-bg);
  border-bottom:.5px solid #f2cf7640;color:var(--warn);font-size:13px;line-height:20px}
#warn span{display:block;max-width:1440px;margin:0 auto}
/* ---- page ---- */
main{padding:0 var(--p);max-width:1440px;margin:0 auto}
.hd{display:flex;align-items:center;gap:var(--g);margin:var(--sp) 0 var(--gap);flex-wrap:wrap}
h2{margin:0;font-size:15px;line-height:20px;font-weight:530;letter-spacing:-.13px;color:var(--fg)}
.hd .hint{margin-left:2px}
h3{margin:0;font-size:13px;line-height:20px;font-weight:530;letter-spacing:-.04px;color:var(--fg)}
.hint{font-size:13px;line-height:20px;font-weight:440;color:var(--muted)}
small{color:var(--faint);font-size:12px}
/* ---- table: surface frame, raised body, hairline rows ---- */
.scroll{overflow:auto;-webkit-overflow-scrolling:touch;background:var(--bg);
  border:.5px solid var(--ring);border-radius:var(--r);height:500px;
  scrollbar-width:thin;scrollbar-color:var(--ring-strong) transparent}
table{border-collapse:separate;border-spacing:0;width:100%;background:transparent;
  font-size:13px;font-variant-numeric:tabular-nums}
thead{background:var(--surface)}
th{position:sticky;top:0;z-index:2;height:32px;background:var(--surface);text-align:left;
  font-size:13px;font-weight:500;color:var(--muted);letter-spacing:-.04px;
  padding:0 12px;border:0;white-space:nowrap;cursor:pointer;user-select:none}
th:hover{color:var(--fg)}
/* sort caret drawn as a mask, so it cannot drift with font metrics */
th::after{content:"";display:inline-block;width:7px;height:10px;margin-left:6px;
  vertical-align:middle;background-color:currentColor;opacity:.4;
  --sc:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='7' height='10'%3E%3Cpath d='M3.5 0 7 4H0zM3.5 10 0 6h7z'/%3E%3C/svg%3E");
  -webkit-mask:var(--sc) center/7px 10px no-repeat;mask:var(--sc) center/7px 10px no-repeat}
th:hover::after{opacity:.7}
th.asc::after{opacity:1;clip-path:inset(0 0 50% 0)}
th.desc::after{opacity:1;clip-path:inset(50% 0 0 0)}
td{height:36px;padding:0 12px;vertical-align:middle;border:0;white-space:nowrap;font-size:13px}
tbody tr:not(:last-child) td{border-bottom:.5px solid var(--ring-muted)}
tbody tr{transition:background-color 85ms ease-out}
tbody tr:hover td{background-color:#ffffff0a}
[data-theme="light"] tbody tr:hover td{background-color:#0000000a}
th.num,td.num{text-align:right}
td:first-child{position:sticky;left:0;z-index:1;background:var(--bg)}
th:first-child{position:sticky;left:0;z-index:3;background:var(--surface)}
tr.hot td,tr.hot td:first-child{background-image:linear-gradient(var(--warn-bg),var(--warn-bg))}
tbody tr.sel td{background-image:linear-gradient(#3b5cf624,#3b5cf624)}
tbody tr.sel td:first-child{box-shadow:inset 2px 0 0 var(--acc)}
tbody tr.stale td{color:var(--faint)}
tbody tr.stale td:first-child{box-shadow:inset 2px 0 0 var(--bad)}
tbody tr.stale.sel td:first-child{box-shadow:inset 2px 0 0 var(--acc)}
/* ---- graph ---- */
#graph{max-width:1440px;margin:0 auto;padding:0 var(--p)}
#graph[hidden]{display:none}
#graph .hd{margin:var(--sp) 0 var(--gap)}
#graph .hd .badge{margin-left:auto}
#gsvg{display:block;width:100%;height:190px;border-radius:var(--r-lg);background:var(--surface);
  box-shadow:var(--elev-raised)}
#glegend{display:flex;flex-wrap:wrap;gap:var(--g) 14px;margin-top:var(--gap);font-size:13px;color:var(--muted)}
#glegend b{color:var(--fg);font-weight:530}
#gclose{height:var(--h-sm);padding:0 9px;border-radius:var(--r-sm);font-size:12px;font-weight:440}
.sw{display:inline-block;width:8px;height:8px;border-radius:2px;margin-right:5px;vertical-align:1px}
.ifi{color:var(--fg);font-weight:530;letter-spacing:-.01em}
.byte{display:inline-block;width:2.1em;color:var(--fg)}
.bts{white-space:nowrap}
.byte.chg{color:var(--warn);font-weight:600}
.age{color:var(--faint);font-size:12px}
.empty{color:var(--faint);padding:24px 12px;text-align:center}
td.empty{height:464px;vertical-align:middle}   /* ponytail: 500px panel - 32px header - 4px frame; hardcode again if the box changes */
#log{display:flex;flex-direction:column}
#log .empty{flex:1;display:flex;align-items:center;justify-content:center}
/* ---- change log ---- */
.ll{display:grid;grid-template-columns:auto minmax(0,1fr);gap:2px var(--g);align-items:baseline;
  padding:8px 12px;border-bottom:.5px solid var(--ring-muted);font-size:13px}
.ll:last-child{border-bottom:0}
.ll .ts{grid-row:1/span 2}
.lrow{display:flex;flex-wrap:wrap;gap:2px var(--g);align-items:baseline;min-width:0}
.lrow .ifi{min-width:2.4em}
.lold,.lold .ifi{color:var(--faint);font-weight:440}
.lold .byte{color:inherit}
.ts{color:var(--faint);font-size:12px;min-width:3.2em;font-variant-numeric:tabular-nums}
/* ---- sampling tabs = segmented control ---- */
#tabs{display:flex;flex-wrap:wrap;gap:2px;align-items:center;padding:8px var(--padx);
  border-bottom:.5px solid var(--ring-muted);background:var(--bg)}
#tabs>*{flex:0 0 auto}
.tab{height:var(--h-sm);padding:0 9px;border:.5px solid transparent;border-radius:var(--r-sm);
  background-color:transparent;
  color:var(--muted);font-size:13px;font-weight:530;box-shadow:none}
.tab:hover{color:var(--fg);background-image:none;background-color:#ffffff0a}
.tab.on{color:var(--fg);background-color:var(--surface);box-shadow:var(--elev-raised)}
.tab.add{color:var(--muted);background-color:transparent;border-color:var(--ring-strong)}
.tab.add:hover{color:var(--fg)}
.tabg{display:inline-flex;align-items:stretch;border-radius:var(--r-sm);overflow:hidden;
  background-color:transparent}
.tabg .tab{border-radius:0;padding:0 6px 0 9px;background-color:transparent;box-shadow:none;color:var(--muted)}
.tabg .tab.on{background-color:var(--surface);color:var(--fg);box-shadow:var(--elev-raised)}
.tabx{border:0;border-left:.5px solid var(--ring-muted);border-radius:0;background-color:transparent;
  color:var(--faint);height:var(--h-sm);padding:0 7px;font-size:11px;box-shadow:none}
.tabx:hover{color:var(--bad);background-color:#ffffff0a}
/* ---- compare ---- */
.cmpbar{display:flex;flex-wrap:wrap;gap:var(--g);align-items:center;margin:0 0 var(--gap)}
.cmpbar .hint{margin-right:auto}
#cmp{font-size:12px}
#cmp th::after{content:none}
#cmp th small{display:block;font-weight:440;font-size:10px;letter-spacing:0;color:var(--faint)}
#cmp td{font-size:12px;padding:0 12px}
#cmp td.cdiff{background-image:linear-gradient(var(--warn-bg),var(--warn-bg));color:var(--warn)}
#cmp td.cmiss{color:var(--faint);font-style:italic}
#cmp tr.same td{color:var(--faint)}
#cmp .pay{font-family:var(--mono)}
#cmp .per{color:var(--faint);margin-left:6px}
/* ---- cards: ring shadow, 8px radius, 16px padding, no border ---- */
#health,#detail,#toolspanel,#dev{max-width:1440px;margin:0 auto;padding:0 var(--p)}
#dev .hgrid{grid-template-columns:repeat(auto-fit,minmax(150px,1fr))}
#health[hidden],#detail[hidden],#toolspanel[hidden]{display:none}
.hgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:var(--gap)}
.card,.cell{display:flex;flex-direction:column;background-color:var(--surface);
  border-radius:var(--r-lg);box-shadow:var(--elev-raised);padding:var(--p);min-width:0}
.card-h{display:flex;align-items:center;justify-content:space-between;gap:12px;
  min-height:20px;margin:0 0 4px}
.card-h h3{margin:0;font-size:13px;font-weight:530;letter-spacing:-.04px;color:var(--fg)}
.cell .k{display:block;margin:0 0 4px;font-size:13px;line-height:20px;font-weight:530;
  letter-spacing:-.04px;color:var(--fg)}
.cell-b,.card-b{margin-top:0;padding:0;min-width:0}
.cell .v{display:block;font-size:24px;line-height:32px;font-weight:610;letter-spacing:-.5px;
  font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.cell .sub{display:block;margin-top:4px;font-size:13px;line-height:20px;font-weight:440;
  color:var(--muted)}

.bar{height:4px;border-radius:2px;background-color:var(--layer2);margin-top:10px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:var(--ok);transition:width .25s,background .25s}
.bar i.warn{background:var(--warn)}.bar i.bad{background:var(--bad)}
.flags{display:flex;flex-wrap:wrap;gap:6px;margin:var(--gap) 0 0}
/* ---- ID detail ---- */
#detail .hd{margin:var(--sp) 0 var(--gap)}
#d-stats{grid-template-columns:repeat(auto-fit,minmax(160px,1fr))}
#d-stats .v{font-size:21px;line-height:1;font-weight:530;letter-spacing:-.37px;margin-top:6px}
/* text metrics get the small size, numbers keep the 24px display size */
.cell .v.txt{font-size:15px;line-height:20px;font-weight:530;letter-spacing:-.13px;
  white-space:normal;font-family:var(--mono)}
.bits{display:grid;gap:4px;border-radius:var(--r-lg);padding:var(--p);margin-top:var(--gap);
  background-color:var(--surface);
  box-shadow:var(--elev-raised);overflow-x:auto}
.brow{display:grid;grid-template-columns:2.2em 2.2em repeat(8,minmax(18px,1fr)) 4.6em;
  gap:4px;align-items:center;font-size:11px;min-width:0}
.bidx{color:var(--muted);font-weight:530}
.bhex{color:var(--fg);font-family:var(--mono);font-size:12px}
.bit{display:flex;align-items:center;justify-content:center;height:20px;border-radius:var(--r-sm);
  border:.5px solid var(--ring-muted);background-color:var(--bg);color:var(--faint);
  font-size:10px;font-weight:530}
.bit.dyn{color:var(--focus);border-color:#3b5cf666;background-color:#3b5cf624}
.bit.one{color:var(--ok);border-color:#6bd5864d;background-color:var(--ok-bg)}
.bit.zero{opacity:.45}
.bcnt{color:var(--faint);font-size:11px;text-align:right;white-space:nowrap}
#dhints{display:flex;flex-direction:column;gap:2px;margin-top:var(--gap)}
/* ---- tools ---- */
.tg{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:var(--gap);
  align-items:start}
.card-b{padding:0}
.card-b p{margin:0 0 8px;font-size:13px;line-height:20px;font-weight:440;color:var(--muted)}
.card-b p:last-child{margin-bottom:0}
.row{display:flex;flex-wrap:wrap;gap:var(--g);align-items:center;margin-top:12px}
.row input{flex:1 1 90px;height:var(--h);padding:0 9px;border-radius:var(--r);
  border:.5px solid var(--ring-muted);background-color:var(--bg);color:var(--fg);
  font-family:inherit;font-size:13px;letter-spacing:-.04px}
.row input:focus{outline:2px solid var(--focus);outline-offset:0}
.row button{height:var(--h)}
#txout:empty{display:none}
#txout{white-space:pre-wrap;font:12px/18px var(--mono);color:var(--muted);margin:12px 0 0;
  max-height:200px;overflow:auto;background-color:var(--bg);border-radius:var(--r-sm);
  padding:8px;border:.5px solid var(--ring-muted)}
@media(max-width:560px){
  .toggles{flex:1 1 auto;margin-right:-11px}
  .tgl{flex:1 1 0}
  .actions button{flex:1 1 0}
  #q{flex:1 1 100%;font-size:16px}
  .cell .v{font-size:21px;line-height:28px;letter-spacing:-.37px;white-space:normal}
  .hide-sm{display:none}
  #sortsel{display:block}
  .hd{margin:20px 0 var(--gap)}
  .hgrid{grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:var(--g)}
  .card,.cell{padding:12px}
  table,tbody{display:block}
  thead{display:none}
  td.empty{height:480px;display:flex;align-items:center;justify-content:center}
  tbody tr{display:grid;grid-template-columns:minmax(0,1fr) auto auto;align-items:baseline;
    column-gap:12px;row-gap:2px;padding:8px 12px;border-bottom:.5px solid var(--ring-muted)}
  tbody tr:last-child{border-bottom:0}
  td{height:auto;padding:0;border:0;white-space:normal}
  td:first-child{position:static;background:none}
  td:nth-child(1){grid-area:1/1;font-size:13px}
  td:nth-child(6){grid-area:1/2;color:var(--faint);font-size:12px}
  td:nth-child(7){grid-area:1/3;color:var(--faint);font-size:12px}
  td:nth-child(6)::before{content:"chg "}
  td:nth-child(3){grid-area:2/1/2/4;font-size:13px}
  td:nth-child(2),td:nth-child(4),td:nth-child(5){display:none}
  tbody tr.hot{background-image:linear-gradient(var(--warn-bg),var(--warn-bg))}
  tbody tr.sel{box-shadow:inset 2px 0 0 var(--acc)}
  tbody tr.stale{box-shadow:inset 2px 0 0 var(--bad)}
  tbody tr.hot td,tbody tr.hot td:first-child{background-image:none;box-shadow:none}
  #gsvg{height:150px}
  .brow{grid-template-columns:1.8em 1.8em repeat(8,minmax(16px,1fr)) 3.6em}
}
@media(min-width:900px){
  main{display:grid;grid-template-columns:minmax(0,1fr) minmax(320px,420px);
    grid-template-rows:auto minmax(0,1fr);column-gap:var(--gap);align-items:start}
  #idshd{grid-area:1/1}
  #idswrap,#cmpscroll{grid-area:2/1}
  #loghd{grid-area:1/2}
  #log{grid-area:2/2}
  #idswrap,#cmpscroll,#log{height:500px}
  main.single{grid-template-columns:minmax(0,1fr)}
}
</style></head><body>
<header>
  <div class="toggles">
    <button id="mode" class="tgl" role="switch" aria-checked="true" aria-label="listen-only mode">
      <span id="mlabel">Listen only</span><span class="track"><span class="knob"></span></span>
    </button>
    <button id="onlytg" class="tgl" role="switch" aria-checked="true">
      <span>Changed only</span><span class="track"><span class="knob"></span></span>
    </button>
  </div>
  <div class="grp">
    <select id="br" aria-label="bitrate">
      <option value="10000">10k</option>
      <option value="20000">20k</option>
      <option value="50000">50k</option>
      <option value="100000">100k</option>
      <option value="125000" selected>125k</option>
      <option value="250000">250k</option>
      <option value="500000">500k</option>
      <option value="1000000">1M</option>
    </select>
    <div class="actions">
      <button id="fz"><span id="fzl">Freeze</span></button>
      <button id="clr">Clear</button>
    </div>
  </div>
  <div id="bars">
    <span class="pill"><i id="dot" class="dot"></i><span id="conn">Offline</span></span>
  </div>
  <select id="theme" aria-label="Theme" title="Theme: system, light or dark">
    <option value="system">System</option>
    <option value="light">Light</option>
    <option value="dark">Dark</option>
  </select>
  <!-- filter last: whatever row it lands on is the row that absorbs the leftover width -->
  <div class="grp grow">
    <input id="q" type="search" placeholder="Filter ID" aria-label="Filter ID" autocomplete="off">
  </div>
</header>
<p id="warn" hidden><span></span></p>
<div id="tabs" role="tablist" aria-label="samples">
  <button class="tab on" id="livetab" role="tab" data-tab="live">Live</button>
  <button class="tab" id="cmptab" role="tab" hidden>Compare</button>
  <button class="tab add" id="snapbtn" title="Store the current ID table as a new tab">+ Sample</button>
</div>
<section id="health">
  <div class="hd"><h2>Bus</h2><span class="badge" id="upt">0s</span></div>
  <div class="hgrid">
    <div class="cell"><span class="k">Load</span>
      <div class="cell-b"><span class="v" id="h-load">0%</span>
        <div class="bar"><i id="h-bar"></i></div></div></div>
    <div class="cell"><span class="k">Frames</span>
      <div class="cell-b"><span class="v" id="h-rate">0 f/s</span>
        <span class="sub" id="h-total">0 frames total</span></div></div>
    <div class="cell"><span class="k">Errors RX / TX</span>
      <div class="cell-b"><span class="v" id="h-err">0 / 0</span></div></div>
    <div class="cell"><span class="k">Timing</span>
      <div class="cell-b"><span class="v" id="h-timing">-</span>
        <span class="sub" id="h-timing-sub"></span></div></div>
    <div class="cell"><span class="k">RX buffer peak</span>
      <div class="cell-b"><span class="v" id="h-rx">0 / 0</span></div></div>
    <div class="cell"><span class="k">Dropped / RTR</span>
      <div class="cell-b"><span class="v" id="h-drop">0 / 0</span></div></div>
  </div>
  <div class="flags" id="h-flags"></div>
</section>
<section id="graph" hidden>
  <div class="hd"><h2>Graph<span id="gid"></span></h2><span class="badge" id="gcount">0 samples</span>
    <button id="gclose" aria-label="close graph">Close</button></div>
  <svg id="gsvg" role="img" aria-label="byte values over time"></svg>
  <div id="glegend"></div>
</section>
<section id="detail" hidden>
  <div class="hd"><h2>ID <span id="d-id"></span><span class="badge" id="d-tag"></span></h2>
    <button id="d-close" aria-label="close detail">Close</button></div>
  <div class="hgrid" id="d-stats"></div>
  <div class="bits" id="d-bits"></div>
  <div id="dhints"></div>
</section>
<main>
  <div class="hd" id="idshd"><h2>IDs</h2><span class="badge" id="icount">0</span>
    <span class="badge" id="snapmeta" hidden></span>
    <select id="sortsel" aria-label="sort by">
      <option value="id:1">ID &#8593;</option>
      <option value="id:-1">ID &#8595;</option>
      <option value="n:-1">Seen &#8595;</option>
      <option value="c:-1">Chg &#8595;</option>
      <option value="pa:1">Period &#8593;</option>
      <option value="ls:1">Age &#8593;</option>
    </select></div>
  <div class="scroll" id="idswrap"><table><thead><tr>
    <th data-k="id" class="asc">ID</th>
    <th data-k="l" class="num hide-sm">Len</th>
    <th>Data</th>
    <th data-k="pa" class="num hide-sm">Period</th>
    <th data-k="n" class="num">Seen</th>
    <th data-k="c" class="num">Chg</th>
    <th data-k="ls" class="num age hide-sm">Age</th>
  </tr></thead><tbody id="ids"></tbody></table></div>
  <div class="hd" id="loghd"><h2>Changes</h2></div>
  <div id="log" class="scroll"></div>
  <div class="scroll" id="cmpscroll" hidden>
    <div class="cmpbar">
      <span class="hint" id="cmpsum"></span>
      <label class="hint"><input type="checkbox" id="cmponly" checked> differences only</label>
      <button id="cmpcsv">Download CSV</button>
    </div>
    <table id="cmp"><thead></thead><tbody></tbody></table>
  </div>
</main>
<section id="toolspanel">
  <div class="hd"><h2>Tools</h2></div>
  <div class="tg">
    <div class="card">
      <div class="card-h"><h3>Export</h3></div>
      <div class="card-b">
        <p>Downloads come from the board's own buffers: the ID table, the raw frame ring
          (newest 384 frames) as CSV, or as a candump log for Wireshark / SavvyCAN / cabana.</p>
        <div class="row">
          <a class="btn" href="/ids.csv">IDs CSV</a>
          <a class="btn" href="/raw.csv">Raw CSV</a>
          <a class="btn" href="/raw.log">candump</a>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-h"><h3>Transmit</h3></div>
      <div class="card-b">
        <p>These controls are there whenever the board is <b>not</b> in listen-only:
          the controller then acknowledges frames and can send OBD-II / UDS requests.
          Flip <b>Listen only</b> off in the top bar to use them, and only on a bus you
          are allowed to write to.</p>
        <p id="txnote" hidden>Transmit is compiled out of this build
          (<code>ALLOW_TX = false</code>), so every request will be refused.</p>
        <div id="txbody" hidden>
          <div class="row">
            <button id="obdall">OBD-II scan</button>
            <button id="vin">VIN</button>
            <button id="dtc">DTCs</button>
          </div>
          <div class="row">
            <input id="reqid" value="7DF" aria-label="request CAN ID" placeholder="7DF">
            <input id="reqd" value="02010C" aria-label="request payload hex" placeholder="02010C">
            <button id="reqgo">Send</button>
          </div>
          <pre id="txout"></pre>
        </div>
      </div>
    </div>
  </div>
</section>
<section id="dev">
  <div class="hd"><h2>Device</h2></div>
  <div class="hgrid">
    <div class="cell"><span class="k">Free heap</span>
      <div class="cell-b"><span class="v" id="s-heap">-</span>
        <span class="sub" id="s-heap2"></span></div></div>
    <div class="cell"><span class="k">Largest block</span>
      <div class="cell-b"><span class="v" id="s-blk">-</span>
        <span class="sub" id="s-blk2"></span></div></div>
    <div class="cell"><span class="k">Loop stack</span>
      <div class="cell-b"><span class="v" id="s-stack">-</span>
        <span class="sub">high-water mark</span></div></div>
    <div class="cell"><span class="k">Chip</span>
      <div class="cell-b"><span class="v txt" id="s-chip">-</span>
        <span class="sub" id="s-chip2"></span></div></div>
    <div class="cell"><span class="k">WiFi</span>
      <div class="cell-b"><span class="v" id="s-wifi">-</span>
        <span class="sub" id="s-wifi2"></span><span class="sub" id="s-wifi3"></span></div></div>
    <div class="cell"><span class="k">Tables</span>
      <div class="cell-b"><span class="v" id="s-tbl">-</span>
        <span class="sub" id="s-tbl2"></span></div></div>
    <div class="cell"><span class="k">MCP2515</span>
      <div class="cell-b"><span class="v txt" id="s-cnf">-</span>
        <span class="sub" id="s-cnf2"></span><span class="sub" id="s-cnf3"></span></div></div>
  </div>
</section>
<script>
const $=sel=>document.querySelector(sel.startsWith('#')?sel:'#'+sel);
const el=(tag,cls,text)=>{const n=document.createElement(tag);if(cls)n.className=cls;if(text!=null)n.textContent=text;return n;};
const NS='http://www.w3.org/2000/svg';
const svgNode=(tag,attrs,text)=>{const n=document.createElementNS(NS,tag);
  for(const k in attrs)n.setAttribute(k,attrs[k]);if(text!=null)n.textContent=text;return n;};
let frozen=false,sortKey='id',sortDir=1,onlyChanged=true,prevTotal=0,prevT=0,rate=0;
let sel=null,hist=[],detailId=null,detailTick=0,modeKnown=null,warnTimer=0;
const HIST=180;   // samples kept: ~54s at the 300ms poll
const S={ids:[],log:[],t:0,lo:1,b:125000,tx:0,atx:1,n:0,h:{}};
let stickyWarn='';
// a message that stays until something else replaces it (mode, transmit state)
function sticky(m){
  stickyWarn=m||'';
  warn(stickyWarn);
}
function warn(m,ms){
  const w=$('warn');
  clearTimeout(warnTimer);
  const text=m||stickyWarn;          // '' falls back to the sticky banner
  if(!text){w.hidden=true;return;}
  w.firstChild.textContent=text;w.hidden=false;
  if(ms)warnTimer=setTimeout(()=>{
    if(stickyWarn){w.firstChild.textContent=stickyWarn;w.hidden=false;}
    else w.hidden=true;
  },ms);
}
// age/delta as short duration: 0.9s / 42s / 1:05
function fmtA(ms){
  ms=Math.max(0,ms);
  if(ms<10000)return (ms/1000).toFixed(1)+'s';
  if(ms<60000)return Math.round(ms/1000)+'s';
  return Math.floor(ms/60000)+':'+String(Math.floor(ms%60000/1000)).padStart(2,'0');
}
// period comes from the board in microseconds
function fmtP(us){
  if(!us)return '-';
  const ms=us/1000;
  if(ms<1)return Math.round(us)+'\u00b5s';
  if(ms<10)return ms.toFixed(2)+'ms';
  if(ms<1000)return Math.round(ms)+'ms';
  return fmtA(ms);
}
const fmtHz=hz=>hz>=1000000?(hz/1000000)+'M':(hz/1000)+'k';
const fmtN=(v,d=0)=>v.toLocaleString(undefined,{minimumFractionDigits:d,maximumFractionDigits:d});

function bytes(d,p){
  if(!d)return el('span','empty','-');
  const wrap=el('span','bts');
  for(let i=0;i<d.length;i+=2){
    const b=d.slice(i,i+2);
    wrap.append(el('span',p&&p.slice(i,i+2)!==b?'byte chg':'byte',b));
  }
  return wrap;
}
function shortId(x){return x.id.toUpperCase().replace(/X$/,'');}

function render(){
  const v=curTab();
  if(v.cmp)return;                       // the compare view draws itself
  const q=$('q').value.trim().toLowerCase().replace(/^0x/,'');
  const list=v.ids.filter(x=>(!onlyChanged||x.c)&&(!q||x.id.toLowerCase().includes(q)));
  const k=sortKey,d=sortDir;
  list.sort((a,b)=>k==='id'
    ? d*((parseInt(a.id,16)||0)-(parseInt(b.id,16)||0))
    : d*((a[k]||0)-(b[k]||0)));
  $('icount').textContent=list.length;
  const rows=[];
  for(const x of list){
    const tr=el('tr');
    tr.dataset.id=x.id;
    const per=x.pa||0;
    const perMs=per/1000;
    const stale=v.live&&perMs>0&&(v.t-x.ls)>Math.max(3000,perMs*4);
    if(v.live&&x.c>0&&(v.t-x.lc)<3000)tr.classList.add('hot');
    if(x.id===sel)tr.classList.add('sel');
    if(stale)tr.classList.add('stale');
    const idc=el('td','ifi',shortId(x));
    if(x.id.endsWith('x'))idc.append(el('span','ext','EXT'));
    const data=el('td');
    data.append(bytes(x.d,x.p));
    const age=el('td','num age hide-sm',((v.t-x.ls)/1000).toFixed(1)+'s');
    if(stale)age.append(el('span','lost','LOST'));
    tr.append(idc,
      el('td','num hide-sm',String(x.l)),
      data,
      el('td','num hide-sm',fmtP(per)),
      el('td','num',String(x.n)),
      el('td','num',String(x.c)),
      age);
    rows.push(tr);
  }
  if(!rows.length){
    const td=el('td','empty',v.ids.length?'No ID matches this filter.'
      :'Waiting for frames. Check the wiring, termination and bitrate.');
    td.colSpan=7;
    const tr=el('tr');
    tr.append(td);
    rows.push(tr);
  }
  $('ids').replaceChildren(...rows);
  if(!v.live)return;                     // stored samples have no change log
  const lines=[];
  for(const l of S.log){
    const line=el('div','ll');
    line.append(el('span','ts',fmtA(S.t-l.t)));
    const now=el('div','lrow');
    if(l.p==='')now.append(el('span','new','New'));
    now.append(el('span','ifi',shortId(l)),bytes(l.d,l.p));
    line.append(now);
    if(l.p!==''){                       // previous value, same columns, muted
      const was=el('div','lrow lold');
      was.append(el('span','ifi',shortId(l)),bytes(l.p,null));
      line.append(was);
    }
    lines.push(line);
  }
  if(!lines.length)lines.push(el('div','empty','No changes yet.'));
  $('log').replaceChildren(...lines);
}

// ---- bus health ----
const EFLAGS=[[1,'EWARN'],[2,'TXWAR'],[4,'RXWAR'],[8,'TXEP'],[16,'RXEP'],[32,'BUS-OFF'],[64,'RX0OVR'],[128,'RX1OVR']];
function renderHealth(){
  const h=S.h||{};
  const load=(h.load||0)/10;
  $('h-load').textContent=load.toFixed(load<10?1:0)+'%';
  const bar=$('h-bar');
  bar.style.width=Math.min(100,load)+'%';
  bar.className=load>80?'bad':load>50?'warn':'';
  $('h-rate').textContent=rate.toFixed(0)+' f/s';
  $('h-total').textContent=(S.n||0).toLocaleString()+(S.n===1?' frame total':' frames total');
  $('h-err').textContent=(h.rec||0)+' / '+(h.tec||0);
  $('h-timing').textContent=h.abr?fmtHz(h.abr):'-';
  $('h-timing-sub').textContent=h.abr?((h.sp||0)/100).toFixed(1)+'% SP \u00b7 '+(h.ppm||0)+' ppm':'';
  $('h-rx').textContent=(h.peak||0)+' / '+(h.rxsz||0);
  $('h-drop').textContent=(h.drop||0)+' / '+(h.rtr||0);
  $('upt').textContent=fmtA(S.t);
  const efl=h.efl||0, cur=h.eflg||0;
  const flags=[];
  for(const [bit,name] of EFLAGS){
    if(efl&bit||cur&bit)flags.push(el('span','flag',name));
  }
  if(h.canok===0)flags.unshift(el('span','flag','CAN DOWN'));
  if((h.peak||0)>=(h.rxsz||1)&&h.rxsz)flags.push(el('span','flag','RX BUFFER FULL'));
  if((h.load||0)>800)flags.push(el('span','flag','HEAVY LOAD'));
  $('h-flags').replaceChildren(...flags);
}

// ---- device resources: ESP32 headroom + MCP2515 timing as programmed ----
function fmtKB(b){b=b||0;return b>=1048576?(b/1048576).toFixed(1)+'M':(b/1024).toFixed(b>=10240?0:1)+'K';}
function renderSys(){
  const s=S.sys||{};
  if(s.heap==null)return;
  $('s-heap').textContent=fmtKB(s.heap);
  $('s-heap2').textContent='min '+fmtKB(s.hmin);
  $('s-blk').textContent=fmtKB(s.blk);
  $('s-blk2').textContent=s.heap?Math.round(s.blk*100/s.heap)+'% of free heap':'';
  $('s-stack').textContent=fmtKB(s.stack);
  $('s-chip').textContent=s.chip||'-';
  $('s-chip2').textContent=(s.cpu||0)+' MHz \u00b7 '+Math.round((s.flash||0)/1024)+' MB flash';
  const linked=s.ip&&s.ip!=='0.0.0.0';
  const ssid=s.ssid||'-';
  if(s.ap){
    const n=s.apcli||0;
    $('s-wifi').textContent=n+(n===1?' client':' clients');
    $('s-wifi2').textContent=(s.ip||'')+(s.apsec?' \u00b7 WPA2':' \u00b7 open');
    $('s-wifi3').textContent='AP \u00b7 '+ssid+' \u00b7 ch '+(s.chan||'-');
  }else if(linked){
    $('s-wifi').textContent=s.rssi+' dBm';
    $('s-wifi2').textContent=s.ip;
    $('s-wifi3').textContent='STA \u00b7 '+ssid+' \u00b7 ch '+(s.chan||'-');
  }else{
    $('s-wifi').textContent='no link';
    $('s-wifi2').textContent='';
    $('s-wifi3').textContent='WiFi off';
  }
  $('s-tbl').textContent=(s.ids||0)+' / '+(s.idsz||0);
  $('s-tbl2').textContent='raw ring '+(s.raw||0)+' / '+(s.rawsz||0);
  const hx=n=>('0'+((n||0)&255).toString(16).toUpperCase()).slice(-2);
  $('s-cnf').textContent=s.cnf1==null?'-':hx(s.cnf1)+' '+hx(s.cnf2)+' '+hx(s.cnf3);
  $('s-cnf2').textContent=((s.quartz||0)/1000)+' MHz \u00b7 SPI '+((s.spi||0)/1000)+' MHz';
  $('s-cnf3').textContent='BRP '+(s.brp||0)+'/TQ '+(s.tq||0)+'/SJW '+(s.sjw||0);
}

// ---- graph: selected ID's bytes over the last HIST polls ----
function drawGraph(){
  const svg=$('gsvg'),W=svg.clientWidth||800,H=190,P=14;
  // palette comes from the theme vars so light/dark graph colours stay readable
  const css=getComputedStyle(document.documentElement),V=n=>css.getPropertyValue(n).trim();
  const GC=[0,1,2,3,4,5,6,7].map(i=>V('--g'+i));
  if(!hist.length){
    $('gcount').textContent='0 samples';
    svg.replaceChildren();
    $('glegend').replaceChildren();
    return;
  }
  const t0=hist[0].t,t1=Math.max(S.t,t0+1);
  $('gcount').textContent=hist.length+(hist.length===1?' sample':' samples')+' \u00b7 '+fmtA(t1-t0);
  const X=t=>P+((t-t0)/(t1-t0))*(W-2*P), Y=v=>H-P-(v/255)*(H-2*P);
  const nodes=[];
  for(const v of [0,128,255]){
    nodes.push(svgNode('line',{x1:P,x2:W-P,y1:Y(v),y2:Y(v),stroke:V('--grid')}));
    nodes.push(svgNode('text',{x:P+2,y:Y(v)-5,fill:V('--dim'),'font-size':10},
      v===255?'FF':v.toString(16).toUpperCase().padStart(2,'0')));
  }
  const leg=[];
  for(let i=0;i<8;i++){
    const pts=[];
    let last='';
    for(const s of hist){
      const b=s.d.slice(i*2,i*2+2);
      if(b.length===2){pts.push(X(s.t).toFixed(1)+','+Y(parseInt(b,16)).toFixed(1));last=b;}
    }
    if(!pts.length)continue;
    nodes.push(svgNode('polyline',{points:pts.join(' '),fill:'none',stroke:GC[i],
      'stroke-width':1.6,'stroke-linejoin':'round','stroke-linecap':'round'}));
    const lxy=pts.at(-1).split(',');
    nodes.push(svgNode('circle',{cx:lxy[0],cy:lxy[1],r:2.5,fill:GC[i]}));
    const sw=el('i','sw');
    sw.style.background=GC[i];
    const item=el('span');
    item.append(sw,document.createTextNode('D'+i+' '),el('b',null,last));
    leg.push(item);
  }
  svg.replaceChildren(...nodes);
  $('glegend').replaceChildren(...leg);
}

// ---- single ID detail: bit map, counter/checksum hints, J1939 fields ----
function j1939(id){
  const v=parseInt(id.toUpperCase().replace(/X$/,''),16);
  const prio=(v>>>26)&7, sa=v&0xFF, dp=(v>>>24)&1, pf=(v>>>16)&0xFF, ps=(v>>>8)&0xFF;
  const pgn=(dp<<16)|(pf<<8)|(pf<240?0:ps);
  return {prio,sa,pgn,dp,pf,ps};
}
function renderDetail(d){
  $('d-id').textContent=shortId(d);
  $('d-tag').textContent=d.l+' bytes'+(d.rtr?' \u00b7 RTR':'');
  const stats=[
    ['Seen',fmtN(d.n)],['Changes',fmtN(d.c)],
    ['Period min',fmtP(d.pm)],['Period avg',fmtP(d.pa)],['Period max',fmtP(d.px)],
    ['Change gap',d.cm?fmtP(d.cm*1000)+' \u2013 '+fmtP(d.cx*1000):'-'],
    ['First seen',fmtA(S.t-d.fs)+' ago'],['Last seen',fmtA(S.t-d.ls)+' ago'],
  ];
  if(d.id.endsWith('x')){
    const j=j1939(d.id);
    stats.push(['J1939','PGN '+j.pgn+' \u00b7 prio '+j.prio+' \u00b7 SA 0x'+j.sa.toString(16).toUpperCase()]);
  }
  $('d-stats').replaceChildren(...stats.map(([k,v])=>{
    const c=el('div','cell');
    const b=el('div','cell-b');
    b.append(el('span','v',v));
    c.append(el('span','k',k),b);
    return c;
  }));
  const rows=[];
  const hex=d.d||'';
  for(let i=0;i<d.l;i++){
    const o=parseInt(d.o.substr(i*2,2),16)||0;
    const a=parseInt(d.a.substr(i*2,2),16)||0;
    const t=parseInt(d.t.substr(i*2,2),16)||0;
    const row=el('div','brow');
    row.append(el('span','bidx','D'+i),el('span','bhex',hex.slice(i*2,i*2+2)||'--'));
    for(let b=7;b>=0;b--){
      const m=1<<b;
      const cls=(t&m)?'dyn':(a&m)?'one':((o&m)?'dyn':'zero');
      const c=el('span','bit '+cls,String(b));
      c.title='D'+i+' bit'+b+((t&m)?' toggles':(a&m)?' stuck 1':' never 1');
      row.append(c);
    }
    row.append(el('span','bcnt',(d.bc[i]||0)+' chg'));
    rows.push(row);
  }
  $('d-bits').replaceChildren(...rows);
  const hints=[];
  if(d.c>=3){
    for(let i=0;i<d.l;i++){
      const share=d.bc[i]/d.c;
      if(share>=0.9)hints.push('D'+i+' changes with '+Math.round(share*100)+'% of payload changes - counter or checksum candidate');
      if(share===0)hints.push('D'+i+' never changed - constant field');
    }
  } else if(d.c===0&&d.n>1){
    hints.push('Payload never changed: static ID');
  }
  let best=0,bi=-1;
  (d.cu||[]).forEach((v,i)=>{if(v>best){best=v;bi=i;}});
  if(best>=3&&d.c>=5&&best>=d.c*0.6){
    const w=2*(bi%4);
    hints.push('16-bit value in D'+w+'..D'+(w+1)+' advances by a small step on '+best+' of '+d.c+
      ' changes ('+(bi<4?'little':'big')+'-endian) - rolling counter candidate');
  }
  const dyn=Array.from({length:d.l},(_,i)=>i).filter(i=>parseInt(d.t.substr(i*2,2),16));
  if(dyn.length)hints.push('Dynamic bytes: '+dyn.map(i=>'D'+i).join(', '));
  $('dhints').replaceChildren(...(hints.length
    ?hints.map(h=>el('div','hint',h))
    :[el('div','hint','No pattern yet.')]));
}
async function loadDetail(id){
  try{
    const r=await fetch('/detail?id='+encodeURIComponent(id));
    if(!r.ok)return;
    renderDetail(await r.json());
  }catch(e){}
}

function selectId(id){
  sel=(sel===id)?null:id;
  hist=[];
  if(sel){
    const e=S.ids.find(x=>x.id===sel);
    if(e)hist.push({t:S.t,d:e.d});
    $('gid').textContent=' \u00b7 '+sel.toUpperCase().replace(/X$/,'');
  }
  $('graph').hidden=!sel;
  drawGraph();
  render();
  detailId=sel;
  $('detail').hidden=!sel;
  if(sel)loadDetail(sel);
}
function setMode(lo){
  $('mode').setAttribute('aria-checked',lo?'true':'false');
  $('mlabel').textContent=lo?'Listen only':'Normal mode';
  if(modeKnown===lo)return;
  modeKnown=lo;
  sticky(lo?'':'Normal mode: the board ACKs frames and the transmit controls below are '
              +'live. Use only on a bus you own.');
}
async function tick(){
  const t0=performance.now();
  try{
    const j=await (await fetch('/data')).json();
    S.ids=j.ids;S.log=j.log;S.t=j.t;S.lo=j.lo;S.b=j.b;S.tx=j.tx;
    S.atx=j.atx===0?0:1;              // missing field means allowed; the board decides anyway
    S.h=j.h||{};S.n=j.n;S.sys=j.sys||{};
    if(document.activeElement!==$('br'))$('br').value=S.b;
    if(prevT)rate=rate*.6+((j.n-prevTotal)/((t0-prevT)/1000))*.4;
    prevTotal=j.n;prevT=t0;
    if(!frozen){$('dot').className='dot ok';$('conn').textContent='Live';}
    setMode(j.lo);
    renderSys();
    if(activeTab!=='live'){renderTx();if(!frozen)setTimeout(tick,300);return;}
    if(sel&&!frozen){
      const e=S.ids.find(x=>x.id===sel);
      if(e){hist.push({t:S.t,d:e.d});if(hist.length>HIST)hist.shift();}
    }
    renderHealth();
    render();
    if(sel)drawGraph();
    if(detailId&&++detailTick%3===0)loadDetail(detailId);
    renderTx();
  }catch(e){
    $('dot').className='dot';$('conn').textContent='Offline';rate=0;
  }
  if(!frozen)setTimeout(tick,300);
}
$('mode').onclick=async()=>{
  $('mode').disabled=true;
  try{
    const r=await (await fetch('/mode?listen='+(S.lo?0:1))).json();
    S.lo=r.lo;S.tx=r.tx;modeKnown=null;setMode(r.lo);
    if(r.lo)sticky(''); else openTxGate();
    if(r.err)warn("Couldn't change mode (error "+r.err+').',8000);
  }catch(e){warn('No response from the board. Mode unchanged.',8000);}
  renderTx();
  $('mode').disabled=false;
};
$('br').onchange=async e=>{
  const hz=+e.target.value;
  $('br').disabled=true;
  warn('');
  try{
    const r=await (await fetch('/bitrate?hz='+hz)).json();
    S.b=r.b;S.tx=0;                        // a bitrate change closes the transmit gate
    if(!r.err&&!S.lo)openTxGate();
    if(r.err)warn("Couldn't switch bitrate (error "+r.err+'). Still at '+fmtHz(S.b)+'.',8000);
    else{
      S.ids=[];S.log=[];S.h={};hist=[];sel=null;detailId=null;
      $('graph').hidden=true;$('detail').hidden=true;
      prevTotal=0;prevT=0;rate=0;render();renderHealth();
    }
  }catch(err){warn('No response from the board. Still at '+fmtHz(S.b)+'.',8000);}
  renderTx();
  $('br').value=S.b;
  $('br').disabled=false;
};
$('fz').onclick=()=>{
  frozen=!frozen;
  $('fzl').textContent=frozen?'Resume':'Freeze';
  $('fz').classList.toggle('on',frozen);
  $('conn').textContent=frozen?'Frozen':'Live';
  $('dot').className='dot '+(frozen?'warn':'ok');
  if(!frozen)setTimeout(tick,300);
};
$('clr').onclick=async()=>{
  await fetch('/clear');
  S.ids=[];S.log=[];S.h={};prevTotal=0;prevT=0;rate=0;hist=[];
  selectId(null);render();renderHealth();
};
$('ids').onclick=e=>{
  const tr=e.target.closest('tr[data-id]');
  if(!tr)return;
  if(activeTab!=='live'){
    warn('Stored sample: read-only. Switch to Live to graph or inspect an ID.',5000);
    return;
  }
  selectId(tr.dataset.id);
};
$('gclose').onclick=()=>selectId(sel);
$('d-close').onclick=()=>selectId(sel);
$('q').oninput=render;
$('sortsel').onchange=e=>{
  const [k,d]=e.target.value.split(':');
  sortKey=k;sortDir=+d;render();
};
$('onlytg').onclick=()=>{
  onlyChanged=!onlyChanged;
  $('onlytg').setAttribute('aria-checked',onlyChanged?'true':'false');
  render();
};

// ---- samples: freeze the current table into a tab, compare the tabs later ----
const SKEY='canwatch-samples';
const MAIN=document.querySelector('main');
let samples=[],activeTab='live',cmpOnly=true;

function loadSamples(){
  try{
    const raw=localStorage.getItem(SKEY);
    if(!raw)return;
    const a=JSON.parse(raw);
    if(Array.isArray(a))samples=a.filter(x=>x&&x.sid&&Array.isArray(x.ids));
  }catch(e){samples=[];}
}
function saveSamples(){
  while(samples.length>8)samples.shift();          // 8 tabs is plenty, and keeps localStorage small
  try{localStorage.setItem(SKEY,JSON.stringify(samples));}
  catch(e){
    while(samples.length>1){
      samples.shift();
      try{localStorage.setItem(SKEY,JSON.stringify(samples));return;}catch(e2){}
    }
    warn('Samples are too large to keep after a reload.',8000);
  }
}
// what the table and log show: the live board, a stored sample, or nothing (compare)
function curTab(){
  if(activeTab==='live')return {ids:S.ids,t:S.t,live:true};
  if(activeTab==='cmp')return {ids:[],t:S.t,live:false,cmp:true};
  const s=samples.find(x=>x.sid===activeTab);
  return s?{ids:s.ids,t:s.boardT,live:false,s}:{ids:[],t:S.t,live:false};
}
function renderTabs(){
  document.querySelectorAll('#tabs .tabg').forEach(n=>n.remove());
  const anchor=$('cmptab');
  for(const s of samples){
    const g=el('span','tabg');
    const b=el('button',activeTab===s.sid?'tab on':'tab',s.name);
    b.title=s.ids.length+' IDs \u00b7 '+fmtN(s.n)+' frames \u00b7 '+fmtHz(s.b);
    b.onclick=()=>setTab(s.sid);
    const x=el('button','tabx','\u00d7');
    x.title='delete this sample';
    x.setAttribute('aria-label','delete '+s.name);
    x.onclick=ev=>{ev.stopPropagation();dropSample(s.sid);};
    g.append(b,x);
    $('tabs').insertBefore(g,anchor);
  }
  $('cmptab').hidden=samples.length<2;
  $('livetab').classList.toggle('on',activeTab==='live');
  $('cmptab').classList.toggle('on',activeTab==='cmp');
}
function setTab(t){
  if(t==='cmp'&&samples.length<2)t='live';
  if(t!=='live'&&t!=='cmp'&&!samples.some(s=>s.sid===t))t='live';
  activeTab=t;
  const live=t==='live',cmp=t==='cmp';
  const s=samples.find(x=>x.sid===t);
  $('health').hidden=!live;
  $('loghd').hidden=!live;
  $('log').hidden=!live;
  $('idswrap').hidden=cmp;
  $('cmpscroll').hidden=!cmp;
  MAIN.classList.toggle('single',!live);
  const meta=$('snapmeta');
  meta.hidden=live||cmp;
  if(s)meta.textContent=s.name+' \u00b7 '+s.ids.length+' IDs \u00b7 '+fmtN(s.n)+
    ' frames \u00b7 '+new Date(s.wall).toLocaleTimeString();
  if(!live){sel=null;detailId=null;$('graph').hidden=true;$('detail').hidden=true;}
  renderTabs();
  if(cmp)renderCompare();else{render();if(live)renderHealth();}
}
function dropSample(sid){
  const i=samples.findIndex(s=>s.sid===sid);
  if(i<0)return;
  const gone=samples.splice(i,1)[0];
  saveSamples();
  if(activeTab===sid)setTab('live');else renderTabs();
  warn('Deleted sample "'+gone.name+'".',4000);
}
// read the whole table, one page at a time, with every per-ID statistic
async function capture(){
  const def='Sample '+(samples.length+1);
  const typed=prompt('Name this sample (the module / device it came from)',def);
  if(typed===null)return;
  const name=(typed||'').trim()||def;
  let from=0,ids=[],meta=null;
  try{
    for(let guard=0;guard<10;guard++){
      const j=await (await fetch('/data?full=1&from='+from+'&count=64')).json();
      if(!meta)meta=j;
      const page=j.ids||[];
      ids=ids.concat(page);
      if(!j.more||!page.length)break;
      from+=page.length;
    }
  }catch(e){warn('Sample failed: no answer from the board.',6000);return;}
  if(!ids.length){warn('Nothing to sample yet: the ID table is empty.',6000);return;}
  const s={sid:Date.now().toString(36),name,wall:Date.now(),boardT:meta.t,n:meta.n,b:meta.b,ids};
  samples.push(s);
  saveSamples();
  setTab(s.sid);
  warn('Sampled "'+name+'": '+ids.length+' IDs, '+fmtN(meta.n)+' frames.',5000);
}
function cmpIds(){
  return [...new Set(samples.flatMap(s=>s.ids.map(x=>x.id)))]
    .sort((a,b)=>(parseInt(a,16)||0)-(parseInt(b,16)||0));
}
function renderCompare(){
  const list=samples;
  const head=el('tr');
  head.append(el('th',null,'ID'));
  for(const s of list){
    const th=el('th',null,s.name);
    th.append(el('small',null,s.ids.length+' IDs \u00b7 '+fmtHz(s.b)+' \u00b7 '+
      new Date(s.wall).toLocaleTimeString()));
    head.append(th);
  }
  $('cmp').querySelector('thead').replaceChildren(head);
  const ids=cmpIds();
  const rows=[];
  let nDiff=0,nSame=0,nOnly=0;
  for(const id of ids){
    const cells=list.map(s=>s.ids.find(x=>x.id===id));
    // a cell compares on payload, length and average period
    const sig=cells.map(c=>c?[c.l,c.d,Math.round((c.pa||0)/1000)].join('|'):'');
    const missing=cells.some(c=>!c);
    const differs=missing||sig.some(x=>x!==sig[0]);
    if(missing)nOnly++;else if(differs)nDiff++;else nSame++;
    if(cmpOnly&&!differs)continue;
    const tr=el('tr',differs?'':'same');
    const idc=el('td','ifi',shortId({id}));
    if(id.endsWith('x'))idc.append(el('span','ext','EXT'));
    tr.append(idc);
    cells.forEach((c,i)=>{
      const td=el('td',!c?'cmiss':(sig[i]!==sig[0]?'cdiff':''));
      if(!c)td.textContent='absent';
      else{
        td.append(el('span','pay',c.l+':'+c.d),el('span','per',fmtP(c.pa)));
      }
      tr.append(td);
    });
    rows.push(tr);
  }
  if(!rows.length){
    const td=el('td','empty',cmpOnly?'All captured samples agree. Untick "differences only" to see everything.'
      :'No ID captured in any sample.');
    td.colSpan=list.length+1;
    const tr=el('tr');
    tr.append(td);
    rows.push(tr);
  }
  $('cmp').querySelector('tbody').replaceChildren(...rows);
  $('cmpsum').textContent=list.length+' samples \u00b7 '+ids.length+' IDs \u00b7 '+nDiff+' differ \u00b7 '+
    nOnly+' missing in some \u00b7 '+nSame+' identical';
}
function downloadCsv(name,text){
  const url=URL.createObjectURL(new Blob([text],{type:'text/csv;charset=utf-8'}));
  const a=el('a');
  a.href=url;
  a.download=name;
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(()=>URL.revokeObjectURL(url),2000);
}
function compareCsv(){
  const lines=['id,sample,len,data,period_avg_us,count,changes,present'];
  for(const id of cmpIds())
    for(const s of samples){
      const c=s.ids.find(x=>x.id===id);
      lines.push([id,s.name.replace(/[,\n]/g,' '),c?c.l:'',c?c.d:'',c?(c.pa||0):'',c?c.n:'',c?c.c:'',c?1:0].join(','));
    }
  downloadCsv('canwatch-compare.csv',lines.join('\n'));
}
$('livetab').onclick=()=>setTab('live');
$('cmptab').onclick=()=>setTab('cmp');
$('snapbtn').onclick=capture;
$('cmponly').onchange=e=>{cmpOnly=e.target.checked;renderCompare();};
$('cmpcsv').onclick=compareCsv;

// ---- tools: exports + gated transmit ----
// the transmit controls are visible exactly when the bus is not in listen-only
function renderTx(){
  const on=!S.lo&&S.atx!==0;
  $('txbody').hidden=!on;
  $('txnote').hidden=S.atx!==0;
  if(!on&&$('txout').textContent)$('txout').textContent='';
}
// leaving listen-only opens the gate too, so no second button is needed
async function openTxGate(){
  if(S.lo||S.atx===0||S.tx)return;
  try{
    const r=await (await fetch('/tx?en=1')).json();
    S.tx=r.tx;
    if(!r.tx)warn('Transmit gate still closed (error '+r.err+').',8000);
  }catch(e){}
  renderTx();
}
function txOut(t){$('txout').textContent=t;}
async function txReq(url){
  txOut('working...');
  try{
    const r=await (await fetch(url)).json();
    if(r.ok===0){txOut('refused: '+r.e);return null;}
    txOut(JSON.stringify(r,null,1));
    return r;
  }catch(e){txOut('no response from the board');return null;}
}

$('obdall').onclick=async()=>{
  const r=await txReq('/obd');
  if(!r||!r.pids)return;
  txOut(r.pids.map(p=>p.n.padEnd(16)+String(p.v).padStart(9)+' '+p.u).join('\n')||'no supported PID answered');
};
$('vin').onclick=async()=>{const r=await txReq('/vin');if(r&&r.ok)txOut('VIN '+r.vin);};
$('dtc').onclick=async()=>{
  const r=await txReq('/dtc');
  if(!r||!r.ok)return;
  txOut('MIL '+(r.mil?'on':'off')+' \u00b7 stored codes '+(r.count!=null?r.count:'?')+'\n'+
    ((r.codes||[]).join(' ')||'none'));
};
$('reqgo').onclick=()=>txReq('/req?id='+encodeURIComponent($('reqid').value.trim())+
  '&data='+encodeURIComponent($('reqd').value.trim()));

document.querySelectorAll('th[data-k]').forEach(th=>{
  th.onclick=()=>{
    const k=th.dataset.k;
    if(sortKey===k)sortDir=-sortDir;else{sortKey=k;sortDir=1;}
    document.querySelectorAll('th[data-k]').forEach(o=>{o.classList.remove('asc','desc');o.removeAttribute('aria-sort');});
    th.classList.add(sortDir>0?'asc':'desc');
    th.setAttribute('aria-sort',sortDir>0?'ascending':'descending');
    $('sortsel').value=k+':'+sortDir;
    render();
  };
});
// ---- theme: device scheme by default, override kept in localStorage ----
const THEME_KEY='canwatch-theme';
const lightMQ=matchMedia('(prefers-color-scheme: light)');
let themeMode=new URLSearchParams(location.search).get('theme')||'system';
try{if(!new URLSearchParams(location.search).get('theme'))themeMode=localStorage.getItem(THEME_KEY)||'system';}catch(e){}
// theme select hugs the visible option (native selects size to the widest one)
const meas=el('span');
meas.style.cssText='position:absolute;left:-9999px;top:0;white-space:pre;visibility:hidden';
document.body.append(meas);
function fitTheme(){
  const s=$('theme'),cs=getComputedStyle(s);
  meas.style.font=cs.font||`${cs.fontWeight} ${cs.fontSize} ${cs.fontFamily}`;
  meas.textContent=s.options[s.selectedIndex].text;
  s.style.width=Math.ceil(meas.offsetWidth+parseFloat(cs.paddingLeft)+parseFloat(cs.paddingRight)+2)+'px';
}
function applyTheme(){
  const light=themeMode==='light'||(themeMode==='system'&&lightMQ.matches);
  document.documentElement.dataset.theme=light?'light':'dark';
  $('theme').value=themeMode;
  document.querySelector('meta[name=theme-color]').content=light?'#ffffff':'#161616';
  fitTheme();
  if(sel)drawGraph();
}
$('theme').onchange=e=>{
  themeMode=e.target.value;
  try{localStorage.setItem(THEME_KEY,themeMode);}catch(err){}
  applyTheme();
};
lightMQ.addEventListener('change',()=>{if(themeMode==='system')applyTheme();});
applyTheme();
loadSamples();
setTab('live');
tick();
</script></body></html>
)rawliteral";

IPAddress ipFrom(const char *s) {
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return IPAddress(0, 0, 0, 0);
  return IPAddress((uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
}

uint32_t ipToU32(IPAddress a) {
  return ((uint32_t)a[0] << 24) | ((uint32_t)a[1] << 16) | ((uint32_t)a[2] << 8) | (uint32_t)a[3];
}

const char *statusName(int s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "idle";
    case WL_NO_SSID_AVAIL:   return "SSID not found (2.4 GHz only, check spelling / hidden SSID)";
    case WL_SCAN_COMPLETED:  return "scan completed";
    case WL_CONNECTED:       return "connected";
    case WL_CONNECT_FAILED:  return "auth failed (wrong password or enterprise mode)";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED:    return "disconnected";
    default:                 return "unknown";
  }
}

// ---------- WiFi reporting ----------
static uint32_t wifiLastCheck = 0, wifiLastBeat = 0, wifiLastRetry = 0, wifiDownSince = 0;
static int wifiLastStatus = -1, wifiApClients = -1;
static uint16_t wifiRetries = 0;
static uint32_t wifiRetryGap = 10000;   // grows to 60 s while the link stays down

const char *modeName(wifi_mode_t m) {
  switch (m) {
    case WIFI_OFF:     return "off";
    case WIFI_STA:     return "STA";
    case WIFI_AP:      return "AP";
    case WIFI_AP_STA:  return "AP+STA";
    default:           return "?";
  }
}

// one block with everything the board knows about its network
void printWifiInfo(const char *tag) {
  const wifi_mode_t mode = WiFi.getMode();
  const int st = WiFi.status();
  Serial.printf("WiFi: %s\n", tag);
  Serial.printf("  mode     %s   mac %s\n", modeName(mode), WiFi.macAddress().c_str());
  if (mode == WIFI_STA || mode == WIFI_AP_STA) {
    if (st == WL_CONNECTED) {
      Serial.printf("  ssid     \"%s\"  channel %d\n", WiFi.SSID().c_str(), (int)WiFi.channel());
      Serial.printf("  ip       %s  mask %s\n", WiFi.localIP().toString().c_str(),
                    WiFi.subnetMask().toString().c_str());
      Serial.printf("  gateway  %s  dns %s\n", WiFi.gatewayIP().toString().c_str(),
                    WiFi.dnsIP().toString().c_str());
      Serial.printf("  url      http://%s\n", WiFi.localIP().toString().c_str());
      Serial.printf("  signal   %ld dBm  bssid %s\n", (long)WiFi.RSSI(), WiFi.BSSIDstr().c_str());
    } else {
      Serial.printf("  station  %d (%s), target \"%s\"\n", st, statusName(st), STA_SSID);
    }
  }
  if (mode == WIFI_AP || mode == WIFI_AP_STA) {
    Serial.printf("  ap       \"%s\"  ip %s  clients %d\n", WiFi.softAPSSID().c_str(),
                  WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum());
  }
  Serial.printf("  heap     %u bytes free\n", (unsigned)ESP.getFreeHeap());
}

// connection events, printed as they happen (WiFi task context)
void onWifiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("WiFi: associated with \"%s\" (channel %d), waiting for an address\n",
                    WiFi.SSID().c_str(), (int)WiFi.channel());
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      printWifiInfo("got an address");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("WiFi: disconnected (status %d: %s)\n", (int)WiFi.status(), statusName(WiFi.status()));
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("AP: client joined");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println("AP: client left");
      break;
    default:
      break;
  }
}

// once a second: log state changes, retry the station link, print a heartbeat
void wifiMonitor() {
  const uint32_t now = (uint32_t)nowMs();
  if (now - wifiLastCheck < 1000) return;
  wifiLastCheck = now;
  const wifi_mode_t mode = WiFi.getMode();
  const int st = WiFi.status();

  if (mode == WIFI_STA) {
    if (st != wifiLastStatus) {
      wifiLastStatus = st;
      Serial.printf("WiFi: status %d (%s) at %lus\n", st, statusName(st), (unsigned long)(now / 1000));
      if (st == WL_CONNECTED) {
        printWifiInfo(wifiDownSince ? "back online" : "connected");
        wifiDownSince = 0;
        wifiRetries = 0;
        wifiRetryGap = 10000;
      } else if (!wifiDownSince) {
        wifiDownSince = now ? now : 1;
      }
    }
    if (st != WL_CONNECTED && STA_ENABLE && strlen(STA_SSID)) {
      if (!wifiRetries || now - wifiLastRetry >= wifiRetryGap) {
        wifiLastRetry = now;
        wifiRetries++;
        Serial.printf("WiFi: reconnect attempt %u, down %lus\n", wifiRetries,
                      (unsigned long)((now - wifiDownSince) / 1000));
        WiFi.reconnect();
        if (wifiRetryGap < 60000) wifiRetryGap *= 2;   // back off, do not spam
      }
    }
  } else if (mode == WIFI_AP || mode == WIFI_AP_STA) {
    const int clients = WiFi.softAPgetStationNum();
    if (clients != wifiApClients) {
      if (wifiApClients >= 0) Serial.printf("AP: clients %d -> %d\n", wifiApClients, clients);
      wifiApClients = clients;
    }
  }

  if (now - wifiLastBeat >= 30000) {
    wifiLastBeat = now;
    String addr = "-";
    if (mode == WIFI_STA && st == WL_CONNECTED) addr = WiFi.localIP().toString();
    else if (mode == WIFI_AP || mode == WIFI_AP_STA) addr = WiFi.softAPIP().toString();
    Serial.printf("up %lus | frames %lu | %s %s | heap %u\n", (unsigned long)(now / 1000),
                  (unsigned long)frameTotal, modeName(mode), addr.c_str(),
                  (unsigned)ESP.getFreeHeap());
  }
}

void printNearbyNetworks() {
  Serial.println("WiFi: scanning for 2.4 GHz networks...");
  const int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("WiFi: no networks found (5 GHz networks are invisible to the ESP32)");
    return;
  }
  for (int i = 0; i < n; i++) {
    Serial.printf("  %-24s ch%-3d %4ld dBm  %s\n", WiFi.SSID(i).c_str(), (int)WiFi.channel(i),
                  (long)WiFi.RSSI(i), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "locked");
  }
  WiFi.scanDelete();
}

// true when the station joined; otherwise the caller falls back to the AP
bool joinStation() {
  if (!STA_ENABLE) return false;
  if (!strlen(STA_SSID)) { Serial.println("WiFi: STA_SSID empty, starting AP"); return false; }

  WiFi.persistent(false);           // do not write credentials to NVS on every boot
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);        // association / address / drop, as they happen
  WiFi.setAutoReconnect(true);      // keep retrying after a drop; wifiMonitor() logs it
  if (!STA_USE_DHCP) {
    const IPAddress ip = ipFrom(STA_IP), gw = ipFrom(STA_GATEWAY), sn = ipFrom(STA_SUBNET);
    const uint32_t mask = ipToU32(sn);
    if (!mask || (ipToU32(ip) & mask) != (ipToU32(gw) & mask) || ipToU32(ip) == ipToU32(gw)) {
      Serial.println("WiFi: STA_IP / STA_GATEWAY / STA_SUBNET disagree, static config is probably wrong");
    }
    WiFi.config(ip, gw, sn);
  }
  if (strlen(STA_USERNAME)) {
    Serial.println("WiFi: enterprise mode (STA_USERNAME set) - clear it for home WiFi");
    WiFi.begin(STA_SSID, WPA2_AUTH_PEAP, STA_USERNAME, STA_USERNAME, STA_PASSWORD);
  } else {
    WiFi.begin(STA_SSID, STA_PASSWORD);
  }
  Serial.printf("WiFi: joining \"%s\" as %s, timeout %us\n", STA_SSID,
                STA_USE_DHCP ? "DHCP" : STA_IP, STA_TIMEOUT_SEC);

  int last = -1;
  for (int i = 0; i < STA_TIMEOUT_SEC * 10 && WiFi.status() != WL_CONNECTED; i++) {
    delay(100);
    if (WiFi.status() != last) {
      last = WiFi.status();
      Serial.printf("  %2us status %d: %s\n", i / 10, last, statusName(last));
    } else if (i % 10 == 9) {
      Serial.printf("  %2us still %d (%s), %ld dBm\n", i / 10 + 1, last, statusName(last),
                    (long)WiFi.RSSI());
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    printWifiInfo("joined");
    return true;
  }
  Serial.printf("WiFi: gave up after %ds, status %d: %s\n",
                STA_TIMEOUT_SEC, (int)WiFi.status(), statusName(WiFi.status()));
  printNearbyNetworks();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi: starting AP instead");
  return false;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ipFrom(AP_IP), ipFrom(AP_GATEWAY), ipFrom(AP_SUBNET));
  const char* ssid = strlen(AP_SSID) ? AP_SSID : "Canwatch";   // empty SSID = default
  if (strlen(AP_PASSWORD) >= 8) {
    WiFi.softAP(ssid, AP_PASSWORD);
  } else {
    if (strlen(AP_PASSWORD)) Serial.println("AP: password under 8 chars, starting open");
    WiFi.softAP(ssid);
  }
  Serial.print("AP: "); Serial.print(ssid);
  Serial.print(strlen(AP_PASSWORD) >= 8 ? "  WPA2  http://" : "  open  http://");
  Serial.println(WiFi.softAPIP());
  printWifiInfo("access point up");
  wifiApClients = WiFi.softAPgetStationNum();
}

// name the ACAN2515 begin() error bits instead of printing a hex code
void printCanError(uint16_t err) {
  static const struct { uint16_t bit; const char *text; } NAMES[] = {
    {ACAN2515::kNoMCP2515, "MCP2515 does not answer on SPI - check CS/SCK/SI/SO, 5V and GND"},
    {ACAN2515::kTooFarFromDesiredBitRate, "requested bitrate cannot be reached with this quartz"},
    {ACAN2515::kInconsistentBitRateSettings, "bit timing segments are inconsistent"},
    {ACAN2515::kINTPinIsNotAnInterrupt, "MCP2515_INT is not an interrupt-capable GPIO"},
    {ACAN2515::kISRIsNull, "interrupt pin configured but no ISR given"},
    {(uint16_t)ACAN2515::kISRNotNullAndNoIntPin, "ISR given but MCP2515_INT is 255 (no INT pin wired)"},
    {ACAN2515::kRequestedModeTimeOut, "controller did not enter the requested mode - quartz or SPI speed"},
    {ACAN2515::kAcceptanceFilterArrayIsNULL, "acceptance filter array is NULL"},
    {ACAN2515::kOneFilterMaskRequiresOneOrTwoAcceptanceFilters, "one filter mask needs 1 or 2 filters"},
    {ACAN2515::kTwoFilterMasksRequireThreeToSixAcceptanceFilters, "two masks need 3 to 6 filters"},
    {ACAN2515::kCannotAllocateReceiveBuffer, "not enough RAM for the RX ring"},
    {ACAN2515::kCannotAllocateTransmitBuffer0, "not enough RAM for TX buffer 0"},
    {ACAN2515::kCannotAllocateTransmitBuffer1, "not enough RAM for TX buffer 1"},
    {ACAN2515::kCannotAllocateTransmitBuffer2, "not enough RAM for TX buffer 2"},
  };
  Serial.printf("CAN init error 0x%04X:\n", err);
  for (const auto &n : NAMES) {
    if (err & n.bit) Serial.printf("  - %s\n", n.text);
  }
  if (!(err & 0x3FFF)) Serial.println("  - unknown error code");
}

// One-shot serial summary of the same numbers the Device section shows.
void printResources() {
  Serial.printf("resources: heap %u free (min %u), largest block %u, loop stack hwm %u, cpu %u MHz\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned)getCpuFrequencyMhz());
  Serial.printf("board: %s rev %u, %u cores, flash %u MB, sketch %u KB\n",
                ESP.getChipModel(), (unsigned)ESP.getChipRevision(), (unsigned)ESP.getChipCores(),
                (unsigned)(ESP.getFlashChipSize() >> 20), (unsigned)(ESP.getSketchSize() >> 10));
  Serial.printf("mcp2515: quartz %u kHz, spi 10000 kHz, CNF %02X %02X %02X, BRP %u, TQ %u, SJW %u, sample point %u.%02u%%\n",
                (unsigned)(QUARTZ_FREQUENCY / 1000), health.cnf1, health.cnf2, health.cnf3,
                (unsigned)health.brp, (unsigned)health.tq, (unsigned)health.sjw,
                (unsigned)(health.sp / 100), (unsigned)(health.sp % 100));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.printf("Canwatch: boot, reset reason %d, heap %u\n", (int)esp_reset_reason(),
                (unsigned)ESP.getFreeHeap());
  Serial.printf("WiFi: STA_ENABLE=%d SSID=%s DHCP=%d timeout=%us\n", STA_ENABLE ? 1 : 0,
                strlen(STA_SSID) ? STA_SSID : "(empty)", STA_USE_DHCP ? 1 : 0, STA_TIMEOUT_SEC);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  ACAN2515Settings settings(QUARTZ_FREQUENCY, bitrate);
  settings.mRequestedMode = ACAN2515Settings::ListenOnlyMode;
  settings.mReceiveBufferSize = 128;
  applyTiming(settings);
  healthBaseUs = nowUs();
  Serial.print("CAN: INT pin ");
  if (MCP2515_INT == 255) Serial.print("255 (polling)"); else Serial.print(MCP2515_INT);
  Serial.printf(", quartz %lu Hz\n", (unsigned long)QUARTZ_FREQUENCY);
  uint16_t err = 0;
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    err = can.begin(settings, CAN_ISR);
    if (!err) break;
    Serial.printf("CAN: begin attempt %u failed\n", attempt);
    delay(300);
  }
  if (err) {
    printCanError(err);
    Serial.println("CAN: continuing without the bus - fix the wiring, then power-cycle or change bitrate");
  } else {
    canOk = true;
    Serial.print("CAN: MCP2515 ok at "); Serial.print(bitrate); Serial.println(" bit/s, listen only");
  }

  if (!joinStation()) startAccessPoint();
  printResources();

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/data", sendData);
  server.on("/detail", handleDetail);
  server.on("/ids.csv", exportCsv);
  server.on("/raw.csv", exportRawCsv);
  server.on("/raw.log", exportCandump);
  server.on("/clear", clearAll);
  server.on("/mode", handleMode);
  server.on("/bitrate", handleBitrate);
  server.on("/tx", handleTx);
  server.on("/req", handleReq);
  server.on("/obd", handleObd);
  server.on("/vin", handleVin);
  server.on("/dtc", handleDtc);
  server.begin();
  Serial.println("HTTP: endpoints /data /detail /ids.csv /raw.csv /raw.log /tx /obd /vin /dtc");
  Serial.println("HTTP: ready");
}

void loop() {
  if (canOk) pumpOnce();
  server.handleClient();
  wifiMonitor();
}
