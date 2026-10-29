#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ACAN2515.h>

// ---------- Settings ----------
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;   // 16 if crystal says 16.000

// ---------- Station ----------
static const bool STA_ENABLE = true;      // join a WiFi network before falling back to the AP
static const bool STA_USE_DHCP = true;     // true = router assigns the address; false = static below
static const char* STA_SSID     = "";
static const char* STA_USERNAME = "";     // WPA2-Enterprise identity; empty = WPA2 personal
static const char* STA_PASSWORD = "";
static const char* STA_IP       = "10.10.90.1";   // must sit inside the router LAN, never the AP address
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
static const byte PIN_SCK = 14, PIN_MISO = 27, PIN_MOSI = 13;

ACAN2515 can(MCP2515_CS, SPI, 255);   // 255 = INT unused, we poll
WebServer server(80);

static bool listenOnly = true;        // default: listen only, never ACK/transmit
static uint32_t bitrate = 125000;
static uint32_t frameTotal = 0;

// ---------- Data tables ----------
static const int MAX_IDS = 128;
struct Entry {
  bool used, ext;
  uint32_t id;
  uint8_t len;
  uint8_t data[8], prev[8];
  uint8_t prevLen;
  uint32_t count, changes, lastChange;
};
static Entry table_[MAX_IDS];

static const int LOG_SIZE = 60;
struct LogLine {
  uint32_t t, id;
  bool ext, isNew;
  uint8_t len, prevLen;
  uint8_t data[8], prev[8];
};
static LogLine logBuf[LOG_SIZE];
static int logHead = 0;
static int logCount = 0;

// re-init driver at current bitrate, keep requested mode
uint16_t applyCan() {
  can.end();
  ACAN2515Settings s(QUARTZ_FREQUENCY, bitrate);
  s.mRequestedMode = listenOnly ? ACAN2515Settings::ListenOnlyMode
                                : ACAN2515Settings::NormalMode;
  return can.begin(s, nullptr);
}

void addLog(const Entry &e, bool isNew) {
  LogLine &l = logBuf[logHead];
  l.t = millis(); l.id = e.id; l.ext = e.ext; l.isNew = isNew;
  l.len = e.len; l.prevLen = e.prevLen;
  memcpy(l.data, e.data, 8); memcpy(l.prev, e.prev, 8);
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

void handleFrame(const CANMessage &f) {
  frameTotal++;
  if (f.rtr) return;
  int freeSlot = -1;
  for (int i = 0; i < MAX_IDS; i++) {
    Entry &e = table_[i];
    if (!e.used) { if (freeSlot < 0) freeSlot = i; continue; }
    if (e.id == f.id && e.ext == f.ext) {
      e.count++;
      if (e.len != f.len || memcmp(e.data, f.data, f.len) != 0) {
        memcpy(e.prev, e.data, 8);
        e.prevLen = e.len;
        e.len = f.len;
        memset(e.data, 0, 8);
        memcpy(e.data, f.data, f.len);
        e.changes++;
        e.lastChange = millis();
        addLog(e, false);
      }
      return;
    }
  }
  if (freeSlot >= 0) {
    Entry &e = table_[freeSlot];
    memset(&e, 0, sizeof(e));
    e.used = true; e.id = f.id; e.ext = f.ext; e.len = f.len;
    memcpy(e.data, f.data, f.len);
    e.count = 1; e.lastChange = millis();
    addLog(e, true);
  }
}

// ---------- JSON helpers ----------
void appendHex(String &s, const uint8_t *d, uint8_t len) {
  char b[3];
  for (uint8_t i = 0; i < len; i++) { sprintf(b, "%02X", d[i]); s += b; }
}

void sendData() {
  String s;
  s.reserve(16000);
  s = "{\"t\":" + String(millis()) + ",\"n\":" + String(frameTotal) + ",\"b\":" + String(bitrate);
  s += ",\"lo\":" + String(listenOnly ? 1 : 0) + ",\"ids\":[";
  bool first = true;
  for (int i = 0; i < MAX_IDS; i++) {
    const Entry &e = table_[i];
    if (!e.used) continue;
    if (!first) s += ",";
    first = false;
    s += "{\"id\":\"" + String(e.id, HEX) + (e.ext ? "x" : "") + "\",\"l\":" + String(e.len);
    s += ",\"d\":\""; appendHex(s, e.data, e.len);
    s += "\",\"p\":\""; if (e.changes) appendHex(s, e.prev, e.prevLen);
    s += "\",\"n\":" + String(e.count) + ",\"c\":" + String(e.changes) + ",\"lc\":" + String(e.lastChange) + "}";
  }
  s += "],\"log\":[";
  first = true;
  for (int k = 0; k < logCount; k++) {
    int idx = (logHead - 1 - k + LOG_SIZE) % LOG_SIZE;   // newest first
    const LogLine &l = logBuf[idx];
    if (!first) s += ",";
    first = false;
    s += "{\"t\":" + String(l.t) + ",\"id\":\"" + String(l.id, HEX) + (l.ext ? "x" : "") + "\",\"d\":\"";
    appendHex(s, l.data, l.len);
    s += "\",\"p\":\""; if (!l.isNew) appendHex(s, l.prev, l.prevLen);
    s += "\"}";
  }
  s += "]}";
  server.send(200, "application/json", s);
}

void clearAll() {
  memset(table_, 0, sizeof(table_));
  logHead = 0; logCount = 0;
  frameTotal = 0;
  server.send(200, "text/plain", "ok");
}

void handleMode() {
  const bool want = server.arg("listen").toInt() != 0;
  uint16_t err = 0;
  if (want != listenOnly) {
    err = can.changeModeOnTheFly(want ? ACAN2515Settings::ListenOnlyMode
                                      : ACAN2515Settings::NormalMode);
    if (err == 0) listenOnly = want;
    Serial.print("CAN mode -> "); Serial.println(listenOnly ? "LISTEN ONLY" : "NORMAL (ACKs frames)");
  }
  server.send(200, "application/json",
              "{\"lo\":" + String(listenOnly ? 1 : 0) + ",\"err\":" + String(err) + "}");
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
    }
    Serial.print("CAN bitrate -> "); Serial.println(bitrate);
  }
  server.send(200, "application/json",
              "{\"b\":" + String(bitrate) + ",\"err\":" + String(err) + "}");
}

// ---------- Web page ----------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0d1117">
<title>Canwatch</title>
<style>
:root{--bg:#0c0c09;--panel:#141410;--panel2:#23231b;--line:#2b2b22;--fg:#e8e8e3;--dim:#abab9c;
  --acc:#e0b25c;--ok:#9bc97a;--bad:#e07070;--warn:#de7c5a;color-scheme:dark;
  --bg2:#12120e;--bg3:#1d1d16;--rowline:#23231b;--bdhover:#474739;--btnact:#2b2b22;--track:#2b2b22;
  --hdr1:#141410;--hdr2:#0e0e0b;--grid:#2a2a21;
  --acc-bg:#e0b25c14;--acc-line:#e0b25c66;--ok-bg:#9bc97a40;--ok-line:#9bc97a66;
  --bad-bg:#e0707014;--bad-line:#e0707066;--warn-bg:#de7c5a12;--warn-line:#de7c5a44;
  --g0:#e0b25c;--g1:#9bc97a;--g2:#de7c5a;--g3:#e07070;--g4:#c0a9e0;--g5:#7fc3c8;--g6:#e39ac0;--g7:#c6cfa8;
  --h:40px;--h-sm:30px;--g:8px;--p:12px;--r:10px;
  --padx:max(var(--p),calc((100% - 1440px) / 2 + var(--p)))}
/* light: Tailwind olive 50-300 neutrals; "system" is resolved to light/dark in JS */
[data-theme="light"]{--bg:#fbfbf9;--panel:#f4f4f0;--panel2:#e8e8e3;--line:#d8d8d0;--fg:#1d1d16;
  --dim:#5b5b4b;--acc:#7a5b10;--ok:#42701f;--bad:#a32f26;--warn:#96501c;color-scheme:light;
  --bg2:#f4f4f0;--bg3:#e8e8e3;--rowline:#e4e4dd;--bdhover:#abab9c;--btnact:#deded6;--track:#d8d8d0;
  --hdr1:#f4f4f0;--hdr2:#edede7;--grid:#e0e0d8;
  --acc-bg:#7a5b1012;--acc-line:#7a5b1044;--ok-bg:#42701f20;--ok-line:#42701f55;
  --bad-bg:#a32f2612;--bad-line:#a32f2655;--warn-bg:#96501c16;--warn-line:#96501c44;
  --g0:#7a5b10;--g1:#42701f;--g2:#96501c;--g3:#a32f26;--g4:#5b3f8a;--g5:#0f6a72;--g6:#8e2f63;--g7:#4a5240}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html{-webkit-text-size-adjust:100%}
body{margin:0;background:var(--bg);color:var(--fg);
  font:14px/1.45 ui-sans-serif,system-ui,sans-serif,"Apple Color Emoji","Segoe UI Emoji","Segoe UI Symbol","Noto Color Emoji";
  font-variant-numeric:tabular-nums;
  padding-bottom:env(safe-area-inset-bottom)}
header{position:sticky;top:0;z-index:5;display:flex;flex-wrap:wrap;gap:var(--g);align-items:center;
  padding:calc(var(--g) + env(safe-area-inset-top)) var(--padx) var(--g);
  background:linear-gradient(180deg,var(--hdr1),var(--hdr2));border-bottom:1px solid var(--line)}
button{display:inline-flex;align-items:center;justify-content:center;gap:var(--g);min-height:var(--h);padding:0 var(--p);
  border-radius:var(--r);border:1px solid var(--line);background:var(--panel2);color:var(--fg);
  font:inherit;font-weight:600;white-space:nowrap;cursor:pointer;transition:border-color .12s,background .12s}
button:hover{border-color:var(--bdhover)}
button:active{border-color:var(--acc);background:var(--btnact)}
/* toggles: label + switch, same height as every other control */
.toggles{display:flex;gap:var(--g);min-width:0}
.tgl{justify-content:space-between;gap:var(--p);font-weight:600;color:var(--dim)}
.tgl .track{position:relative;flex:0 0 auto;width:30px;height:16px;border-radius:999px;
  border:1px solid var(--line);background:var(--track);transition:.15s}
.tgl .knob{position:absolute;top:1px;left:1px;width:12px;height:12px;border-radius:50%;
  background:var(--dim);transition:left .15s,background .15s}
.tgl[aria-checked="true"]{color:var(--fg)}
.tgl[aria-checked="true"] .knob{left:15px}
.tgl[aria-checked="true"] .track{background:var(--ok-bg);border-color:var(--ok-line)}
.tgl[aria-checked="true"] .knob{background:var(--ok)}
#mode[aria-checked="false"]{color:var(--bad);border-color:var(--bad-line);background:var(--bad-bg)}
#mode[aria-checked="false"] .track{background:var(--bad-line);border-color:var(--bad-line)}
#mode[aria-checked="false"] .knob{left:1px;background:var(--bad)}
#fz.on{color:var(--acc);border-color:var(--acc-line);background:var(--acc-bg)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--bad);flex:0 0 auto;transition:.2s}
.dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok-line)}
.dot.warn{background:var(--warn);box-shadow:0 0 8px var(--warn-line)}
/* header is one flex-wrap row: fixed-fit controls keep their width, buttons and the filter fill the rest */
header>*{flex:1 1 auto;min-width:0}
.toggles,.tgl,#bars,#theme,#br{flex:0 0 auto}
.grp{display:flex;gap:var(--g);align-items:center;min-width:0}
.grp.grow{flex:2 1 110px}
.actions{display:flex;gap:var(--g);flex:1 1 auto}
.actions button{flex:1 1 0}
#bars{display:flex;gap:var(--g);align-items:center;flex-wrap:wrap}
.pill{display:inline-flex;align-items:center;gap:6px;height:var(--h);padding:0 var(--p);border-radius:var(--r);
  background:var(--bg);border:1px solid var(--line);color:var(--dim);font-size:14px;line-height:1.2;white-space:nowrap}
#q{flex:1 1 100%;min-width:120px;min-height:var(--h);padding:0 var(--p);border-radius:var(--r);
  border:1px solid var(--line);background:var(--bg);color:var(--fg);font:inherit;font-size:16px}
#q:focus,#br:focus,#sortsel:focus,#theme:focus{outline:none;border-color:var(--acc);box-shadow:0 0 0 3px var(--acc-bg)}
select{appearance:none;-webkit-appearance:none;cursor:pointer;background-repeat:no-repeat;
  background-position:right var(--p) center;background-size:10px 6px;
  background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='10' height='6'%3E%3Cpath d='M1 1l4 4 4-4' fill='none' stroke='%238b949e' stroke-width='1.6' stroke-linecap='round' stroke-linejoin='round'/%3E%3C/svg%3E")}
[data-theme="light"] select{background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='10' height='6'%3E%3Cpath d='M1 1l4 4 4-4' fill='none' stroke='%23656d76' stroke-width='1.6' stroke-linecap='round' stroke-linejoin='round'/%3E%3C/svg%3E")}
#br,#theme{min-height:var(--h);padding:0 calc(var(--p) + 16px) 0 var(--p);border-radius:var(--r);
  border:1px solid var(--line);background-color:var(--panel2);color:var(--fg);font:inherit;font-size:16px}
#theme{padding-right:calc(var(--p) + 18px)}
#sortsel{display:none;margin-left:auto;min-height:var(--h-sm);padding:0 calc(var(--g) + 14px) 0 var(--g);
  border-radius:var(--r);border:1px solid var(--line);background-color:var(--panel2);color:var(--dim);font:inherit;font-size:13px}
#rate::before{content:"\00b7";margin:0 6px;color:var(--line)}
#warn{margin:0;padding:var(--g) var(--p);background:var(--warn-bg);border-bottom:1px solid var(--warn-line);
  color:var(--warn);font-size:13px;line-height:1.5}
#warn span{display:block;max-width:64ch}
main{padding:0 var(--p) 40px;max-width:1440px;margin:0 auto}
.hd{display:flex;align-items:center;gap:var(--g);margin:var(--p) 0 var(--g)}
h2{margin:0;font-size:13px;letter-spacing:.02em;color:var(--dim);font-weight:700}
.badge{font-size:11px;color:var(--acc);background:var(--acc-bg);border:1px solid var(--acc-line);
  border-radius:999px;padding:0 8px;line-height:1.6}
.hint{font-size:12px;color:var(--dim)}
.scroll{overflow:auto;-webkit-overflow-scrolling:touch;border:1px solid var(--line);
  border-radius:10px;background:var(--bg2);max-height:min(56vh,620px)}
table{border-collapse:collapse;width:100%}
th{position:sticky;top:0;z-index:2;background:var(--bg3);text-align:left;font-size:11px;
  letter-spacing:.06em;text-transform:uppercase;color:var(--dim);font-weight:700;
  padding:9px var(--p);border-bottom:1px solid var(--line);white-space:nowrap;
  cursor:pointer;user-select:none}
th:hover{color:var(--acc)}
th.asc::after{content:" \25B2"}th.desc::after{content:" \25BC"}
td{padding:9px var(--p);border-bottom:1px solid var(--rowline);white-space:nowrap;font-size:13px}
tbody tr:last-child td{border-bottom:0}
th.num,td.num{text-align:right}
td:first-child,th:first-child{position:sticky;left:0;z-index:1;background:var(--bg2)}
th:first-child{z-index:3;background:var(--bg3)}
tr.hot td{background:var(--warn-bg)}
tr.hot td:first-child{background:var(--warn-bg);box-shadow:inset 3px 0 0 var(--warn)}
tr.hot.sel td:first-child{box-shadow:none}
tbody tr.sel{background:var(--acc-bg);box-shadow:inset 3px 0 0 var(--acc)}
#graph{max-width:1440px;margin:0 auto;padding:0 var(--p) 4px}
#graph[hidden]{display:none}
#graph .hd{margin:var(--g) 0}
#graph .hd .badge{margin-left:auto}
#gsvg{display:block;width:100%;height:190px;border:1px solid var(--line);border-radius:10px;background:var(--bg2)}
#glegend{display:flex;flex-wrap:wrap;gap:var(--g) 14px;margin-top:var(--g);font-size:12px;color:var(--dim)}
#glegend b{color:var(--fg);font-weight:600}
#gclose{min-height:var(--h-sm);padding:0 var(--g);font-size:12px;font-weight:500;color:var(--dim)}
.sw{display:inline-block;width:8px;height:8px;border-radius:2px;margin-right:5px;vertical-align:1px}
.ifi{color:var(--acc);font-weight:600}
.ext{font-size:10px;color:var(--warn);border:1px solid var(--warn-line);border-radius:4px;
  padding:0 3px;margin-left:5px;font-weight:700;vertical-align:1px}
.byte{display:inline-block;width:2.1em;color:var(--fg)}
.bts{white-space:nowrap}
.byte.chg{color:var(--bad);font-weight:700}
.age{color:var(--dim);font-size:12px}
.empty{color:var(--dim);padding:16px var(--p);text-align:center}
.ll{display:grid;grid-template-columns:auto minmax(0,1fr);gap:3px var(--g);align-items:baseline;
  padding:var(--g) var(--p);border-bottom:1px solid var(--rowline);font-size:13px}
.ll .ts{grid-row:1/span 2}
.lrow{display:flex;flex-wrap:wrap;gap:2px var(--g);align-items:baseline;min-width:0}
.lrow .ifi{min-width:2.4em}
/* previous value: same layout as the current one, only muted */
.lold,.lold .ifi{color:var(--dim);font-weight:500}
.lold .byte{color:inherit}
.ll:last-child{border-bottom:0}
.ts{color:var(--dim);font-size:12px;min-width:3.4em;font-variant-numeric:tabular-nums}
.new{font-size:10px;color:var(--bg);background:var(--ok);border-radius:4px;padding:0 4px;font-weight:700}
small{color:var(--dim)}
@media(max-width:560px){
  /* phones: the toggles sit alone on the first line, so let them fill it */
  .toggles{flex:1 1 auto}
  .tgl{flex:1 1 0}
  #rate{display:none}
  .hide-sm{display:none}
  #sortsel{display:block}
  .hd{margin:var(--p) 0 var(--g)}
  /* IDs table becomes two-line rows: no horizontal scrolling on phones */
  .scroll{max-height:min(52vh,560px)}
  table,tbody{display:block}
  thead{display:none}
  tbody tr{display:grid;grid-template-columns:minmax(0,1fr) auto auto;align-items:baseline;
    column-gap:var(--p);row-gap:3px;padding:9px var(--p);border-bottom:1px solid var(--rowline)}
  tbody tr:last-child{border-bottom:0}
  td{padding:0;border:0;white-space:normal}
  td:first-child{position:static;background:none}
  td:nth-child(1){grid-area:1/1;font-size:14px}
  td:nth-child(4){grid-area:1/2;color:var(--dim);font-size:12px}
  td:nth-child(5){grid-area:1/3;color:var(--dim);font-size:12px}
  td:nth-child(4)::before{content:"seen "}
  td:nth-child(5)::before{content:"chg "}
  td:nth-child(3){grid-area:2/1/2/4;font-size:13px}
  td:nth-child(2),td:nth-child(6){display:none}
  tbody tr.hot{background:var(--warn-bg);box-shadow:inset 3px 0 0 var(--warn)}
  tbody tr.sel{box-shadow:inset 3px 0 0 var(--acc)}
  tbody tr.hot td,tbody tr.hot td:first-child{background:none;box-shadow:none}
  #gsvg{height:150px}
}
@media(min-width:900px){
  /* wide screens: live table and change log side by side, no dead space */
  main{display:grid;grid-template-columns:minmax(0,1fr) minmax(320px,420px);
    grid-template-rows:auto minmax(0,1fr);column-gap:16px;align-items:start}
  main>.hd:nth-child(1){grid-area:1/1}
  main>.scroll{grid-area:2/1}
  main>.hd:nth-child(3){grid-area:1/2}
  #log{grid-area:2/2}
  main>.scroll,#log{max-height:calc(100dvh - 185px)}
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
      <button id="fz">Freeze</button>
      <button id="clr">Clear</button>
    </div>
  </div>
  <div id="bars">
    <span class="pill"><i id="dot" class="dot"></i><span id="conn">Offline</span><span id="rate">0 f/s</span></span>
    <span class="pill hide-sm" id="cnt">0</span>
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
<section id="graph" hidden>
  <div class="hd"><h2>Graph<span id="gid"></span></h2><span class="badge" id="gcount">0 samples</span>
    <button id="gclose" aria-label="close graph">Close</button></div>
  <svg id="gsvg" role="img" aria-label="byte values over time"></svg>
  <div id="glegend"></div>
</section>
<main>
  <div class="hd"><h2>IDs</h2><span class="badge" id="icount">0</span>
    <span class="hint">Select a row to graph</span>
    <select id="sortsel" aria-label="sort by">
      <option value="id:1">ID &#8593;</option>
      <option value="id:-1">ID &#8595;</option>
      <option value="n:-1">Seen &#8595;</option>
      <option value="c:-1">Chg &#8595;</option>
      <option value="lc:1">Age &#8593;</option>
    </select></div>
  <div class="scroll"><table><thead><tr>
    <th data-k="id" class="asc">ID</th>
    <th data-k="l" class="num hide-sm">Len</th>
    <th>Data</th>
    <th data-k="n" class="num">Seen</th>
    <th data-k="c" class="num">Chg</th>
    <th data-k="lc" class="num age hide-sm">Age</th>
  </tr></thead><tbody id="ids"></tbody></table></div>
  <div class="hd"><h2>Changes</h2></div>
  <div id="log" class="scroll"></div>
</main>
<script>
const $=sel=>document.querySelector(sel.startsWith('#')?sel:'#'+sel);
const el=(tag,cls,text)=>{const n=document.createElement(tag);if(cls)n.className=cls;if(text!=null)n.textContent=text;return n;};
const NS='http://www.w3.org/2000/svg';
const svgNode=(tag,attrs,text)=>{const n=document.createElementNS(NS,tag);
  for(const k in attrs)n.setAttribute(k,attrs[k]);if(text!=null)n.textContent=text;return n;};
let frozen=false,sortKey='id',sortDir=1,onlyChanged=true,prevTotal=0,prevT=0,rate=0;
let sel=null,hist=[];
const HIST=180;   // samples kept: ~54s at the 300ms poll
const S={ids:[],log:[],t:0,lo:1,b:125000};
const warn=m=>{const w=$('warn');w.hidden=!m;w.firstChild.textContent=m||'';};
// age/delta as short duration: 0.9s / 42s / 1:05
function fmtA(ms){
  ms=Math.max(0,ms);
  if(ms<10000)return (ms/1000).toFixed(1)+'s';
  if(ms<60000)return Math.round(ms/1000)+'s';
  return Math.floor(ms/60000)+':'+String(Math.floor(ms%60000/1000)).padStart(2,'0');
}
const fmtHz=hz=>hz>=1000000?(hz/1000000)+'M':(hz/1000)+'k';

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
  const q=$('q').value.trim().toLowerCase().replace(/^0x/,'');
  const list=S.ids.filter(x=>(!onlyChanged||x.c)&&(!q||x.id.toLowerCase().includes(q)));
  const k=sortKey,d=sortDir;
  list.sort((a,b)=>k==='id'
    ? d*((parseInt(a.id,16)||0)-(parseInt(b.id,16)||0))
    : d*((a[k]||0)-(b[k]||0)));
  $('icount').textContent=list.length;
  const rows=[];
  for(const x of list){
    const tr=el('tr');
    tr.dataset.id=x.id;
    if(x.c>0&&(S.t-x.lc)<3000)tr.classList.add('hot');
    if(x.id===sel)tr.classList.add('sel');
    const idc=el('td','ifi',shortId(x));
    if(x.id.endsWith('x'))idc.append(el('span','ext','EXT'));
    const data=el('td');
    data.append(bytes(x.d,x.p));
    tr.append(idc,
      el('td','num hide-sm',String(x.l)),
      data,
      el('td','num',String(x.n)),
      el('td','num',String(x.c)),
      el('td','num age hide-sm',((S.t-x.lc)/1000).toFixed(1)+'s'));
    rows.push(tr);
  }
  if(!rows.length){
    const td=el('td','empty',S.ids.length?'No ID matches this filter.'
      :'Waiting for frames. Check the wiring and bitrate.');
    td.colSpan=6;
    const tr=el('tr');
    tr.append(td);
    rows.push(tr);
  }
  $('ids').replaceChildren(...rows);
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
}
function setMode(lo,err){
  $('mode').setAttribute('aria-checked',lo?'true':'false');
  $('mlabel').textContent=lo?'Listen only':'Normal mode';
  if(err)warn("Couldn't change mode (error "+err+').');
  else warn(lo?'':'Normal mode ACKs frames and can transmit. Use only when safe.');
}
async function tick(){
  const t0=performance.now();
  try{
    const j=await (await fetch('/data')).json();
    S.ids=j.ids;S.log=j.log;S.t=j.t;S.lo=j.lo;S.b=j.b;
    if(document.activeElement!==$('br'))$('br').value=S.b;
    if(prevT)rate=rate*.6+((j.n-prevTotal)/((t0-prevT)/1000))*.4;
    prevTotal=j.n;prevT=t0;
    $('cnt').textContent=j.n.toLocaleString()+(j.n===1?' frame':' frames');
    $('rate').textContent=rate.toFixed(0)+' f/s';
    if(!frozen){$('dot').className='dot ok';$('conn').textContent='Live';}
    setMode(j.lo,0);
    if(sel&&!frozen){
      const e=S.ids.find(x=>x.id===sel);
      if(e){hist.push({t:S.t,d:e.d});if(hist.length>HIST)hist.shift();}
    }
    render();
    if(sel)drawGraph();
  }catch(e){
    $('dot').className='dot';$('conn').textContent='Offline';rate=0;
  }
  if(!frozen)setTimeout(tick,300);
}
$('mode').onclick=async()=>{
  $('mode').disabled=true;
  try{
    const r=await (await fetch('/mode?listen='+(S.lo?0:1))).json();
    S.lo=r.lo;setMode(r.lo,r.err);
  }catch(e){warn('No response from the board. Mode unchanged.');}
  $('mode').disabled=false;
};
$('br').onchange=async e=>{
  const hz=+e.target.value;
  $('br').disabled=true;
  warn('');
  try{
    const r=await (await fetch('/bitrate?hz='+hz)).json();
    S.b=r.b;
    if(r.err)warn("Couldn't switch bitrate (error "+r.err+'). Still at '+fmtHz(S.b)+'.');
    else{S.ids=[];S.log=[];hist=[];sel=null;$('graph').hidden=true;prevTotal=0;prevT=0;rate=0;render();}
  }catch(err){warn('No response from the board. Still at '+fmtHz(S.b)+'.');}
  $('br').value=S.b;
  $('br').disabled=false;
};
$('fz').onclick=()=>{
  frozen=!frozen;
  $('fz').textContent=frozen?'Resume':'Freeze';
  $('fz').classList.toggle('on',frozen);
  $('conn').textContent=frozen?'Frozen':'Live';
  $('dot').className='dot '+(frozen?'warn':'ok');
  if(!frozen)setTimeout(tick,300);
};
$('clr').onclick=async()=>{
  await fetch('/clear');
  S.ids=[];S.log=[];prevTotal=0;prevT=0;rate=0;selectId(sel);render();
};
$('ids').onclick=e=>{
  const tr=e.target.closest('tr[data-id]');
  if(tr)selectId(tr.dataset.id);
};
$('gclose').onclick=()=>selectId(sel);
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
  document.querySelector('meta[name=theme-color]').content=light?'#fbfbf9':'#0c0c09';
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
  Serial.printf("WiFi: joining \"%s\" as %s\n", STA_SSID, STA_USE_DHCP ? "DHCP" : STA_IP);

  int last = -1;
  for (int i = 0; i < STA_TIMEOUT_SEC * 10 && WiFi.status() != WL_CONNECTED; i++) {
    delay(100);
    if (WiFi.status() != last) {
      last = WiFi.status();
      Serial.printf("  status %d: %s\n", last, statusName(last));
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi: http://%s  rssi %ld dBm\n", WiFi.localIP().toString().c_str(), (long)WiFi.RSSI());
    return true;
  }
  Serial.printf("WiFi: gave up after %ds, status %d: %s\n",
                STA_TIMEOUT_SEC, WiFi.status(), statusName(WiFi.status()));
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
}

void setup() {
  Serial.begin(115200);
  delay(500);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  ACAN2515Settings settings(QUARTZ_FREQUENCY, bitrate);
  settings.mRequestedMode = ACAN2515Settings::ListenOnlyMode;
  const uint16_t err = can.begin(settings, nullptr);
  if (err != 0) {
    Serial.print("CAN init error 0x"); Serial.println(err, HEX);
    while (true) delay(1000);
  }

  if (!joinStation()) startAccessPoint();

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/data", sendData);
  server.on("/clear", clearAll);
  server.on("/mode", handleMode);
  server.on("/bitrate", handleBitrate);
  server.begin();
}

void loop() {
  can.poll();
  CANMessage f;
  while (can.receive(f)) handleFrame(f);
  server.handleClient();
}
