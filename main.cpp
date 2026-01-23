#include <Arduino.h>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  #include <LittleFS.h>
  #define WebServer ESP8266WebServer
#else
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <Preferences.h>
#endif
#include <Wire.h>
#include <time.h>

#include <DNSServer.h>

#include <si5351.h>
#include <JTEncode.h>
#ifdef HAS_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif

#ifdef HAS_NEOPIXEL
// ---------- LED SETTINGS ----------
#define LED_PIN 48
Adafruit_NeoPixel rgb(1, LED_PIN, NEO_GRBW + NEO_KHZ800);
#endif

// ---------- I2C PINS ----------
#ifndef I2C_SDA
#define I2C_SDA 21  // fallback
#endif
#ifndef I2C_SCL
#define I2C_SCL 22  // fallback
#endif

// ---------- HOSTNAME ----------
#ifdef ESP8266
  static const char* HOSTNAME = "ESP8266WSPR";   // -> http://ESP8266WSPR.local/
#else
  static const char* HOSTNAME = "ESP32WSPR";   // -> http://ESP32WSPR.local/
#endif

// ---------- WSPR CONSTANTS ----------
static const double TONE_SPACING_HZ = 1.4648;
// WSPR symbol is ~0.682666s. Use micros() scheduling to avoid drift/overrun.
static const uint32_t SYMBOL_PERIOD_US = 683000UL;

static const uint32_t SI5351_CRYSTAL = 25000000UL;

// ---------- DEFAULTS ----------
static const char* DEFAULT_CALL = "N0CALL";
static const char* DEFAULT_LOC  = "ZZ00";
static const uint8_t DEFAULT_PWR_DBM = 10;

static const char* DEFAULT_NTP_SERVER = "pool.ntp.org";

// ---------- Band table (WSPR dial frequencies) ----------
struct BandDef { const char* name; double dial_hz; };
static const BandDef BANDS[] = {
  {"160m",  1836600.0},
  {"80m",   3568600.0},
  {"60m",   5287200.0},
  {"40m",   7038600.0},
  {"30m",  10138700.0},
  {"20m",  14095600.0},
  {"17m",  18104600.0},
  {"15m",  21094600.0},
  {"12m",  24924600.0},
  {"10m",  28124600.0},
  {"6m",   50293000.0},
  {"2m",  144488500.0}
};
static const size_t NUM_BANDS = sizeof(BANDS) / sizeof(BANDS[0]);

// ---------- GLOBALS ----------
Si5351 si5351;
JTEncode jt;
uint8_t symbols[162];

WebServer server(80);
#ifndef ESP8266
Preferences prefs;
#endif

// Captive portal DNS
DNSServer dnsServer;
static const byte DNS_PORT = 53;
bool captivePortalActive = false;

// Settings (loaded from NVS)
String wifiSsid;
String wifiPass;

String CALLSIGN;
String LOCATOR;
uint8_t POWER_DBM;

size_t bandIndex = 3; // default 40m

// per-band calibration offsets (Hz)
double bandCalHz[NUM_BANDS];

// per-band clock output assignment
uint8_t bandClockOut[NUM_BANDS];

// TX control
bool txEnabled = false;      // default OFF
bool txEverySlot = false;    // default alternate

// Si5351 clock output selection
uint8_t si5351Clock = SI5351_CLK0;  // default CLK0 (kept for backward compatibility)

#ifdef HAS_NEOPIXEL
// LED control
bool ledEnabled = true;      // default ON
bool isTxActive = false;     // track TX state
#endif

// NTP server
String ntpServer = DEFAULT_NTP_SERVER;

// per-TX random offset in Hz within window
double sessionFreqOffsetHz = 0.0;

// ---------- Helpers ----------
static String htmlEscape(const String& s) {
  String o; o.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += c; break;
    }
  }
  return o;
}

static bool timeValid() {
  time_t now; time(&now);
  return (now > 1000000000); // sanity threshold
}

// Place carrier near middle of 200 Hz WSPR window: dial + 100 Hz
static double wsprBaseHz() {
  return BANDS[bandIndex].dial_hz + 100.0;
}

static String keyCalForBand(size_t idx) {
  // keys: cal0, cal1, ... cal10
  return "cal" + String((int)idx);
}

// ---------- LED CONTROL ----------
void ledOff() {
#ifdef HAS_NEOPIXEL
  rgb.setPixelColor(0, 0, 0, 0);
  rgb.show();
#endif
}
void ledIdle() {
#ifdef HAS_NEOPIXEL
  if (ledEnabled) {
    rgb.setPixelColor(0, 0, 20, 0);
    rgb.show();
  }
#endif
}
void ledTx() {
#ifdef HAS_NEOPIXEL
  if (ledEnabled) {
    rgb.setPixelColor(0, 20, 0, 0);
    rgb.show();
  }
#endif
}

// ---------- RF CONTROL ----------
void rfOff() {
  si5351.output_enable((si5351_clock)bandClockOut[bandIndex], 0);
  si5351.set_freq(0, (si5351_clock)bandClockOut[bandIndex]);
  Serial.println("RF state: OFF");
#ifdef HAS_NEOPIXEL
  isTxActive = false;
#endif
  ledIdle();
}
void rfOn() {
  si5351.output_enable((si5351_clock)bandClockOut[bandIndex], 1);
  Serial.println("RF state: ON");
#ifdef HAS_NEOPIXEL
  isTxActive = true;
#endif
  ledTx();
}

// ---------- NVS LOAD/SAVE ----------
void loadSettings() {
  // Default per-band calibration (Hz)
  bandCalHz[0]  =  0.0;   // 160m
  bandCalHz[1]  =  0.0;   // 80m
  bandCalHz[2]  =  0.0;   // 60m
  bandCalHz[3]  =  600.0; // 40m
  bandCalHz[4]  =  0.0;   // 30m
  bandCalHz[5]  =  0.0;   // 20m
  bandCalHz[6]  =  0.0;   // 17m
  bandCalHz[7]  =  0.0;   // 15m
  bandCalHz[8]  =  0.0;   // 12m
  bandCalHz[9]  =  0.0;   // 10m
  bandCalHz[10] =  0.0;   // 6m
  bandCalHz[11] =  0.0;   // 2m

  // Default per-band clock outputs (all CLK0)
  for (size_t i = 0; i < NUM_BANDS; i++) {
    bandClockOut[i] = SI5351_CLK0;
  }

#ifdef ESP8266
  LittleFS.begin();
  if (LittleFS.exists("/config.txt")) {
    File f = LittleFS.open("/config.txt", "r");
    if (f) {
      wifiSsid = f.readStringUntil('\n'); wifiSsid.trim();
      wifiPass = f.readStringUntil('\n'); wifiPass.trim();
      CALLSIGN = f.readStringUntil('\n'); CALLSIGN.trim();
      LOCATOR = f.readStringUntil('\n'); LOCATOR.trim();
      POWER_DBM = f.readStringUntil('\n').toInt();
      bandIndex = f.readStringUntil('\n').toInt();
      if (bandIndex >= NUM_BANDS) bandIndex = 3;
      for (size_t i = 0; i < NUM_BANDS; i++) {
        bandCalHz[i] = f.readStringUntil('\n').toDouble();
      }
      for (size_t i = 0; i < NUM_BANDS; i++) {
        bandClockOut[i] = f.readStringUntil('\n').toInt();
      }
      txEnabled = f.readStringUntil('\n').toInt();
      txEverySlot = f.readStringUntil('\n').toInt();
      si5351Clock = f.readStringUntil('\n').toInt();
#ifdef HAS_NEOPIXEL
      ledEnabled = f.readStringUntil('\n').toInt();
#endif
      ntpServer = f.readStringUntil('\n'); ntpServer.trim();
      f.close();
    }
  }
  if (CALLSIGN.isEmpty()) CALLSIGN = DEFAULT_CALL;
  if (LOCATOR.isEmpty()) LOCATOR = DEFAULT_LOC;
  if (ntpServer.isEmpty()) ntpServer = DEFAULT_NTP_SERVER;
#else
  prefs.begin("esp32wspr", true);

  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");

  CALLSIGN  = prefs.getString("call", DEFAULT_CALL);
  LOCATOR   = prefs.getString("loc",  DEFAULT_LOC);
  POWER_DBM = (uint8_t)prefs.getUChar("pwr", DEFAULT_PWR_DBM);

  bandIndex = (size_t)prefs.getUChar("band", 3);
  if (bandIndex >= NUM_BANDS) bandIndex = 3;

  // per-band calibration (override defaults)
  for (size_t i = 0; i < NUM_BANDS; i++) {
    String k = keyCalForBand(i);
    if (prefs.isKey(k.c_str())) bandCalHz[i] = prefs.getDouble(k.c_str(), bandCalHz[i]);
  }

  // per-band clock outputs
  for (size_t i = 0; i < NUM_BANDS; i++) {
    String k = "clkout" + String((int)i);
    if (prefs.isKey(k.c_str())) {
      bandClockOut[i] = prefs.getUChar(k.c_str(), SI5351_CLK0);
    }
  }

  txEnabled   = prefs.getBool("txen", false);    // default OFF
  txEverySlot = prefs.getBool("txall", false);   // default alternate
  si5351Clock = prefs.getUChar("clk", SI5351_CLK0); // default CLK0
#ifdef HAS_NEOPIXEL
  ledEnabled  = prefs.getBool("leden", true);    // default ON
#endif
  ntpServer   = prefs.getString("ntp", DEFAULT_NTP_SERVER);

  prefs.end();
#endif
}

void saveSettings() {
#ifdef ESP8266
  File f = LittleFS.open("/config.txt", "w");
  if (f) {
    f.println(wifiSsid);
    f.println(wifiPass);
    f.println(CALLSIGN);
    f.println(LOCATOR);
    f.println(POWER_DBM);
    f.println(bandIndex);
    for (size_t i = 0; i < NUM_BANDS; i++) {
      f.println(bandCalHz[i], 1);
    }
    for (size_t i = 0; i < NUM_BANDS; i++) {
      f.println(bandClockOut[i]);
    }
    f.println(txEnabled ? 1 : 0);
    f.println(txEverySlot ? 1 : 0);
    f.println(si5351Clock);
#ifdef HAS_NEOPIXEL
    f.println(ledEnabled ? 1 : 0);
#endif
    f.println(ntpServer);
    f.close();
  }
#else
  prefs.begin("esp32wspr", false);

  prefs.putString("ssid", wifiSsid);
  prefs.putString("pass", wifiPass);

  prefs.putString("call", CALLSIGN);
  prefs.putString("loc",  LOCATOR);
  prefs.putUChar("pwr", POWER_DBM);

  prefs.putUChar("band", (uint8_t)bandIndex);

  for (size_t i = 0; i < NUM_BANDS; i++) {
    String k = keyCalForBand(i);
    prefs.putDouble(k.c_str(), bandCalHz[i]);
  }

  // Save per-band clock outputs
  for (size_t i = 0; i < NUM_BANDS; i++) {
    String k = "clkout" + String((int)i);
    prefs.putUChar(k.c_str(), bandClockOut[i]);
  }

  prefs.putBool("txen", txEnabled);
  prefs.putBool("txall", txEverySlot);
  prefs.putUChar("clk", si5351Clock);
#ifdef HAS_NEOPIXEL
  prefs.putBool("leden", ledEnabled);
#endif
  prefs.putString("ntp", ntpServer);

  prefs.end();
#endif
}

// ---------- WIFI + NTP ----------
bool connectStaWithTimeout(uint32_t timeoutMs) {
  if (wifiSsid.isEmpty()) {
    Serial.println("No stored SSID; skipping STA connect.");
    return false;
  }

  WiFi.mode(WIFI_AP_STA);
#ifdef ESP8266
  WiFi.hostname(HOSTNAME);
#else
  WiFi.setHostname(HOSTNAME);
#endif
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

  Serial.printf("Connecting STA to '%s' (timeout %lus)\n", wifiSsid.c_str(), timeoutMs / 1000);

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("STA connected: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nSTA connect timed out.");
  return false;
}

void startApModeCaptivePortal() {
  WiFi.mode(WIFI_AP_STA);
  const char* apSsid = "TechMinds-ESP32WSPR";
  const char* apPass = ""; // open AP

  bool ok = WiFi.softAP(apSsid, apPass);
  Serial.printf("AP %s: %s\n", ok ? "started" : "FAILED", apSsid);
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  captivePortalActive = true;
  Serial.println("Captive portal DNS started");
}

bool syncNtpTime(uint32_t timeoutMs = 20000) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("NTP: no STA connection; cannot sync time yet.");
    return false;
  }

  configTime(0, 0, ntpServer.c_str());

  Serial.printf("NTP: syncing via %s", ntpServer.c_str());
  uint32_t start = millis();
  time_t now = 0;
  while (millis() - start < timeoutMs) {
    time(&now);
    if (now > 1000000000) {
      Serial.println(" ok");
      return true;
    }
    Serial.print(".");
    delay(300);
  }
  Serial.println(" failed");
  return false;
}

// ---------- TX slot schedule ----------
time_t computeNextTxEpoch(time_t now) {
  time_t t = ((now / 120) + 1) * 120;  // next WSPR slot

  if (!txEverySlot) {
    while (true) {
      int minute = (t / 60) % 60;
      int slotIndex = (minute / 2);   // 0,1,2,...
      if ((slotIndex % 2) == 0) break;
      t += 120;
    }
  }
  return t;
}

// ---------- WEB UI ----------
static String pageHtml() {
  // Embedded HTML; location panel removed; GPS removed.
  String html =
R"HTML(<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Tech Minds ESP32WSPR</title>
<style>
  :root{
    --bg:#0b1220; --panel:#101a2e; --panel2:#0f172a;
    --txt:#e5e7eb; --muted:#94a3b8; --acc:#38bdf8; --good:#34d399; --bad:#fb7185;
    --br:#22304a;
  }
  body{ margin:0; font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial; background:var(--bg); color:var(--txt); }
  header{ padding:16px 18px; border-bottom:1px solid var(--br); background:linear-gradient(180deg,var(--panel),#0b1220); }
  h1{ margin:0; font-size:18px; letter-spacing:.2px; }
  .sub{ color:var(--muted); font-size:13px; margin-top:6px; }
  .wrap{ max-width:1020px; margin:0 auto; padding:16px; }
  .grid{ display:grid; grid-template-columns:1fr; gap:14px; }
  @media(min-width:900px){ .grid{ grid-template-columns:1fr 1fr; } }
  .card{ background:var(--panel2); border:1px solid var(--br); border-radius:14px; padding:14px; box-shadow:0 8px 20px rgba(0,0,0,.25); }
  .card h2{ margin:0 0 10px 0; font-size:15px; color:#dbeafe; }
  label{ display:block; font-size:12px; color:var(--muted); margin:10px 0 6px; }
  input,select{
    width:100%; box-sizing:border-box; padding:10px 10px; border-radius:10px;
    border:1px solid var(--br); background:#0b1430; color:var(--txt); outline:none;
  }
  input:focus,select:focus{ border-color:rgba(56,189,248,.55); box-shadow:0 0 0 3px rgba(56,189,248,.12); }
  .row{ display:grid; grid-template-columns:1fr 1fr; gap:10px; }
  button{
    padding:10px 12px; border-radius:12px; border:1px solid rgba(56,189,248,.35);
    background:rgba(56,189,248,.12); color:var(--txt); cursor:pointer; font-weight:700;
  }
  button:hover{ background:rgba(56,189,248,.18); }
  .btnline{ display:flex; gap:10px; align-items:center; flex-wrap:wrap; margin-top:12px; }
  .pill{ padding:6px 10px; border-radius:999px; background:#0b1430; border:1px solid var(--br); color:var(--muted); font-size:12px; }
  .ok{ color:var(--good); } .no{ color:var(--bad); }
  pre{ background:#07102a; border:1px solid var(--br); padding:10px; border-radius:12px; overflow:auto; }
  small{ color:var(--muted); }

  /* toggle */
  .tog { display:flex; align-items:center; justify-content:space-between; gap:10px; padding:10px; border:1px solid var(--br); border-radius:12px; background:#0b1430; }
  .switch { position:relative; width:52px; height:28px; }
  .switch input { display:none; }
  .slider{
    position:absolute; inset:0; background:#172554; border:1px solid rgba(56,189,248,.25);
    border-radius:999px; transition:.2s;
  }
  .slider:before{
    content:""; position:absolute; height:22px; width:22px; left:3px; top:2px;
    background:#e5e7eb; border-radius:50%; transition:.2s;
  }
  .switch input:checked + .slider{ background:rgba(52,211,153,.18); border-color:rgba(52,211,153,.35); }
  .switch input:checked + .slider:before{ transform:translateX(24px); }

  .topbar{ margin-top:12px; display:grid; grid-template-columns:1fr; gap:10px; }
  @media(min-width:900px){ .topbar{ grid-template-columns:1fr 1fr; } }
  .topitem{ background:#0b1430; border:1px solid var(--br); border-radius:14px; padding:10px; }
  .topitem .k{ color:var(--muted); font-size:12px; }
  .topitem .v{ font-size:14px; margin-top:4px; }
  .big{ font-size:16px; font-weight:800; }

  /* Band panel */
  .bandTable{ width:100%; border-collapse:separate; border-spacing:0 8px; }
  .bandRow{ background:#0b1430; border:1px solid var(--br); border-radius:12px; }
  .bandRow td{ padding:10px; vertical-align:middle; }
  .bandRow td:first-child{ width:52px; text-align:center; }
  .bandRow td:nth-child(2){ width:70px; font-weight:800; }
  .bandRow td:nth-child(3){ color:var(--muted); }
  .bandRow td:nth-child(4){ width:160px; }
  .bandRow td:nth-child(5){ width:90px; }
  .bandActive{ outline:2px solid rgba(56,189,248,.35); box-shadow:0 0 0 3px rgba(56,189,248,.08); }
  .radio{ width:18px; height:18px; accent-color: #38bdf8; }
  .calInput{ width:100%; }
  .clkSelect{ width:100%; }
  details summary{
    cursor:pointer; user-select:none; font-weight:800; color:#dbeafe; list-style:none;
  }
  details summary::-webkit-details-marker{ display:none; }
  .summaryLine{
    display:flex; align-items:center; justify-content:space-between;
    gap:10px; padding:10px 12px; border:1px solid var(--br);
    border-radius:12px; background:#0b1430;
  }
  .chev{ color:var(--muted); font-weight:800; }
</style>
</head>
<body>
<header>
  <h1>Tech Minds ESP32WSPR</h1>
  <div class="sub">Configure Wi-Fi + WSPR settings • Hostname: <b>)HTML" + String(HOSTNAME) + R"HTML(.local</b></div>

  <div class="topbar">
    <div class="topitem">
      <div class="k">Time</div>
      <div class="v big" id="timeUtc">UTC: —</div>
      <div class="v"><small id="timeSrc">Source: NTP</small></div>
    </div>
    <div class="topitem">
      <div class="k">Next transmit</div>
      <div class="v big" id="countdown">—</div>
      <div class="v"><small id="txState">—</small></div>
    </div>
  </div>
</header>

<div class="wrap">
  <div class="grid">
    <div class="card">
      <h2>Wi-Fi</h2>

      <div class="btnline">
        <button type="button" onclick="scan()">Scan Networks</button>
        <span class="pill" id="wifiState">Loading…</span>
      </div>

      <label>SSID</label>
      <select id="ssidSel"></select>

      <label>Password</label>
      <input id="pass" type="password" placeholder="(leave blank if open)"/>

      <div class="btnline">
        <button type="button" onclick="saveWifi()">Save Wi-Fi</button>
        <small>Reboot after changing Wi-Fi.</small>
      </div>

      <label>NTP server</label>
      <input id="ntp" placeholder="pool.ntp.org" />

      <div class="btnline">
        <button type="button" onclick="saveNtp()">Save NTP</button>
        <button type="button" onclick="syncTime()">Sync Time Now</button>
      </div>

      <label>Si5351 Clock Output</label>
      <select id="clkout">
        <option value="0">CLK0</option>
        <option value="1">CLK1</option>
        <option value="2">CLK2</option>
      </select>

      <div class="btnline">
        <button type="button" onclick="saveClk()">Save Clock</button>
      </div>

)HTML"
#ifdef HAS_NEOPIXEL
+ R"HTML(
      <label>Status LED</label>
      <div class="tog">
        <div>
          <div class="big">NeoPixel On/Off</div>
        </div>
        <label class="switch">
          <input id="leden" type="checkbox"/>
          <span class="slider"></span>
        </label>
      </div>

      <div class="btnline">
        <button type="button" onclick="saveLed()">Save LED</button>
      </div>
)HTML"
#endif
+ R"HTML(
    </div>

    <div class="card">
      <h2>WSPR Settings</h2>

      <div class="row">
        <div>
          <label>Callsign</label>
          <input id="call" maxlength="6"/>
        </div>
        <div>
          <label>Locator</label>
          <input id="loc" maxlength="4"/>
        </div>
      </div>

      <div class="row">
        <div>
          <label>Power (dBm)</label>
          <input id="pwr" type="number" min="0" max="60" />
        </div>
        <div>
          <label>&nbsp;</label>
          <div class="pill">Per-band calibration below</div>
        </div>
      </div>

      <label>Bands & per-band calibration (Hz)</label>
      <div id="bandPanel">Loading bands…</div>

      <label>Transmit control</label>
      <div class="tog">
        <div>
          <div class="big">TX Enabled</div>
          <small>OFF by default for safety</small>
        </div>
        <label class="switch">
          <input id="txen" type="checkbox"/>
          <span class="slider"></span>
        </label>
      </div>

      <div class="tog" style="margin-top:10px;">
        <div>
          <div class="big">TX Every Slot</div>
          <small>OFF = alternate slots (every 4 minutes)</small>
        </div>
        <label class="switch">
          <input id="txall" type="checkbox"/>
          <span class="slider"></span>
        </label>
      </div>

      <div class="btnline">
        <button type="button" onclick="saveWspr()">Save WSPR</button>
      </div>
    </div>

    <div class="card" style="grid-column:1/-1;">
      <details id="statusDetails">
        <summary>
          <div class="summaryLine">
            <span>Status (advanced)</span>
            <span class="chev" id="statusChev">▶</span>
          </div>
        </summary>
        <div style="margin-top:10px;">
          <pre id="status">Loading…</pre>
          <div class="btnline">
            <button type="button" onclick="refresh(true)">Refresh</button>
            <button type="button" onclick="reboot()">Reboot</button>
          </div>
        </div>
      </details>
    </div>

  </div>
</div>

<script>
let last = null;

// Smooth time: server epoch + (now - fetch_ms)
let serverEpochAtFetch = 0;
let fetchMs = 0;

// avoid overwriting the form every refresh
let formLocked = false;

function fmt2(n){ return String(n).padStart(2,'0'); }
function fmtHMS(sec){
  if(sec < 0) sec = 0;
  const m = Math.floor(sec/60), s = Math.floor(sec%60);
  return `${fmt2(m)}:${fmt2(s)}`;
}
function fmtTimeUTC(epoch){
  const d = new Date(epoch*1000);
  return `${fmt2(d.getUTCHours())}:${fmt2(d.getUTCMinutes())}:${fmt2(d.getUTCSeconds())}`;
}
function currentUtcEpoch(){
  if(!last || !last.time_valid) return 0;
  const dt = (Date.now() - fetchMs) / 1000.0;
  return Math.floor(serverEpochAtFetch + dt);
}

function wireFormLock(){
  const ids = ['call','loc','pwr','txen','txall',)HTML"
#ifdef HAS_NEOPIXEL
+ R"HTML('leden',)HTML"
#endif
+ R"HTML('ntp'];
  ids.forEach(id=>{
    const el = document.getElementById(id);
    el.addEventListener('input', ()=>{ formLocked = true; });
    el.addEventListener('change', ()=>{ formLocked = true; });
  });
}
function updateStatusChevron(){
  const d = document.getElementById('statusDetails');
  const c = document.getElementById('statusChev');
  c.textContent = d.open ? '▼' : '▶';
}
document.getElementById('statusDetails').addEventListener('toggle', updateStatusChevron);

function buildBandPanel(){
  const host = document.getElementById('bandPanel');
  if(!last || !last.bands) { host.textContent = 'No band data.'; return; }

  const tbl = document.createElement('table');
  tbl.className = 'bandTable';

  last.bands.forEach((b, idx)=>{
    const tr = document.createElement('tr');
    tr.className = 'bandRow' + (b.active ? ' bandActive' : '');

    const tdRadio = document.createElement('td');
    const radio = document.createElement('input');
    radio.type = 'radio';
    radio.name = 'activeBand';
    radio.className = 'radio';
    radio.value = String(idx);
    radio.checked = !!b.active;
    radio.addEventListener('change', ()=>{
      formLocked = true;
      [...tbl.querySelectorAll('.bandRow')].forEach(r=>r.classList.remove('bandActive'));
      tr.classList.add('bandActive');
    });
    tdRadio.appendChild(radio);

    const tdName = document.createElement('td');
    tdName.textContent = b.name;

    const tdFreq = document.createElement('td');
    tdFreq.textContent = `${(b.dial_hz/1e6).toFixed(6)} MHz (dial)`;

    const tdCal = document.createElement('td');
    const cal = document.createElement('input');
    cal.type = 'number';
    cal.step = '0.1';
    cal.className = 'calInput';
    cal.id = `cal_${idx}`;
    cal.value = (b.cal_hz ?? 0);
    cal.addEventListener('input', ()=>{ formLocked = true; });
    tdCal.appendChild(cal);

    const tdClk = document.createElement('td');
    const clkSel = document.createElement('select');
    clkSel.id = `clkout_${idx}`;
    clkSel.className = 'clkSelect';
    ['CLK0', 'CLK1', 'CLK2'].forEach((name, clkIdx)=>{
      const opt = document.createElement('option');
      opt.value = String(clkIdx);
      opt.textContent = name;
      opt.selected = (b.clk_out ?? 0) === clkIdx;
      clkSel.appendChild(opt);
    });
    clkSel.addEventListener('change', ()=>{ formLocked = true; });
    tdClk.appendChild(clkSel);

    tr.appendChild(tdRadio);
    tr.appendChild(tdName);
    tr.appendChild(tdFreq);
    tr.appendChild(tdCal);
    tr.appendChild(tdClk);
    tbl.appendChild(tr);
  });

  host.innerHTML = '';
  host.appendChild(tbl);
}

function fillFormOnce(){
  if(formLocked) return;
  document.getElementById('call').value = last.call || '';
  document.getElementById('loc').value = last.loc || '';
  document.getElementById('pwr').value = last.pwr_dbm ?? 10;
  document.getElementById('txen').checked = !!last.tx_enabled;
  document.getElementById('txall').checked = !!last.tx_every_slot;
)HTML"
#ifdef HAS_NEOPIXEL
+ R"HTML(
  document.getElementById('leden').checked = !!last.led_enabled;
)HTML"
#endif
+ R"HTML(
  document.getElementById('ntp').value = last.ntp_server || 'pool.ntp.org';
  document.getElementById('clkout').value = String(last.si5351_clock ?? 0);
  buildBandPanel();
}

function updateTopPanel(){
  if(!last) return;

  if(last.time_valid){
    const now = currentUtcEpoch();
    document.getElementById('timeUtc').textContent = `UTC: ${fmtTimeUTC(now)}`;
  } else {
    document.getElementById('timeUtc').textContent = `UTC: (waiting for time)`;
  }

  document.getElementById('timeSrc').textContent = `Source: NTP (${last.ntp_server || 'pool.ntp.org'})`;
}

function tickCountdown(){
  if(!last) return;

  const txState = document.getElementById('txState');
  const cd = document.getElementById('countdown');

  if(!last.tx_enabled){
    cd.textContent = 'TX DISABLED';
    txState.textContent = last.tx_every_slot ? 'Every slot' : 'Alternate slots';
    return;
  }
  if(!last.time_valid){
    cd.textContent = 'WAITING FOR TIME';
    txState.textContent = 'TX will start once time is valid';
    return;
  }

  const now = currentUtcEpoch();
  const remain = (last.next_tx_epoch || 0) - now;
  const activeBand = (last.band || '—');
  txState.textContent = (last.tx_every_slot ? 'Every slot' : 'Alternate slots') + ` • Band ${activeBand}`;
  cd.textContent = `Next TX in ${fmtHMS(remain)} (at ${fmtTimeUTC(last.next_tx_epoch)} UTC)`;
}

async function refresh(forceFill=false){
  const r = await fetch('/status');
  last = await r.json();

  if(last.time_valid){
    serverEpochAtFetch = last.now_epoch || 0;
    fetchMs = Date.now();
  }

  document.getElementById('status').textContent = JSON.stringify(last, null, 2);

  const st = document.getElementById('wifiState');
  if(last.sta_connected){
    st.textContent = 'STA: ' + last.sta_ip;
    st.className = 'pill ok';
  } else {
    st.textContent = 'AP mode available';
    st.className = 'pill no';
  }

  if(forceFill){
    formLocked = false;
  }
  fillFormOnce();
  updateTopPanel();
  tickCountdown();
}

async function scan(){
  const sel = document.getElementById('ssidSel');
  sel.innerHTML = '<option>Scanning…</option>';
  const r = await fetch('/scan');
  const j = await r.json();
  sel.innerHTML = '';
  (j.networks || []).forEach(n=>{
    const o = document.createElement('option');
    o.value = n.ssid;
    o.textContent = `${n.ssid}  (${n.rssi} dBm)`;
    sel.appendChild(o);
  });
  if(!sel.options.length){
    sel.innerHTML = '<option>(no networks found)</option>';
  }
}

async function saveWifi(){
  const ssid = document.getElementById('ssidSel').value || '';
  const pass = document.getElementById('pass').value || '';
  const body = new URLSearchParams({ssid, pass});
  await fetch('/save_wifi', {method:'POST', body});
  await refresh(true);
  alert('Saved Wi-Fi. Reboot to try connecting.');
}

async function saveNtp(){
  const ntp = document.getElementById('ntp').value || 'pool.ntp.org';
  const body = new URLSearchParams({ntp});
  await fetch('/save_ntp', {method:'POST', body});
  await refresh(true);
  alert('Saved NTP server.');
}

async function syncTime(){
  await fetch('/sync_time', {method:'POST'});
  await refresh(true);
}

async function saveClk(){
  const clk = document.getElementById('clkout').value || '0';
  const body = new URLSearchParams({clk});
  await fetch('/save_clk', {method:'POST', body});
  await refresh(true);
  alert('Saved clock output.');
}

function getActiveBandIndex(){
  const r = document.querySelector('input[name="activeBand"]:checked');
  return r ? r.value : null;
}

async function saveWspr(){
  const call = document.getElementById('call').value || '';
  const loc  = document.getElementById('loc').value || '';
  const pwr  = document.getElementById('pwr').value || '10';
  const txen = document.getElementById('txen').checked ? '1' : '0';
  const txall = document.getElementById('txall').checked ? '1' : '0';

  const band = getActiveBandIndex();
  if(band === null){
    alert('Select an active band first.');
    return;
  }

  const body = new URLSearchParams({call, loc, pwr, txen, txall, band});

  if(last && last.bands){
    last.bands.forEach((b, idx)=>{
      const el = document.getElementById(`cal_${idx}`);
      const v = el ? (el.value || '0') : '0';
      body.append(`cal_${idx}`, v);
      
      const clkEl = document.getElementById(`clkout_${idx}`);
      const clkV = clkEl ? (clkEl.value || '0') : '0';
      body.append(`clkout_${idx}`, clkV);
    });
  }

  await fetch('/save_wspr', {method:'POST', body});
  formLocked = false;
  await refresh(true);
  alert('Saved WSPR settings.');
}

)HTML"
#ifdef HAS_NEOPIXEL
+ R"HTML(
async function saveLed(){
  const leden = document.getElementById('leden').checked ? '1' : '0';
  const body = new URLSearchParams({leden});
  await fetch('/save_led', {method:'POST', body});
  formLocked = false;
  await refresh(true);
  alert('Saved LED setting.');
}

)HTML"
#endif
+ R"HTML(
async function reboot(){
  await fetch('/reboot', {method:'POST'});
  alert('Rebooting…');
}

setInterval(()=>{ updateTopPanel(); tickCountdown(); }, 1000);
setInterval(()=>refresh(false), 10000);

(async ()=>{
  wireFormLock();
  updateStatusChevron();
  await refresh(true);
  await scan();
})();
</script>
</body>
</html>)HTML";

  return html;
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", pageHtml());
}

void handleCaptivePortal() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "text/html; charset=utf-8", pageHtml());
}

void handleStatus() {
  bool sta = (WiFi.status() == WL_CONNECTED);

  time_t now; time(&now);
  bool tOk = (now > 1000000000);
  if (!tOk) now = 0;

  time_t nextTx = tOk ? computeNextTxEpoch(now) : 0;

  String json = "{";
  json += "\"hostname\":\"" + String(HOSTNAME) + "\",";
  json += "\"sta_connected\":" + String(sta ? "true" : "false") + ",";
  json += "\"sta_ip\":\"" + (sta ? WiFi.localIP().toString() : "") + "\",";
  json += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\",";

  json += "\"call\":\"" + htmlEscape(CALLSIGN) + "\",";
  json += "\"loc\":\"" + htmlEscape(LOCATOR) + "\",";
  json += "\"pwr_dbm\":" + String(POWER_DBM) + ",";
  json += "\"band\":\"" + String(BANDS[bandIndex].name) + "\",";
  json += "\"band_index\":" + String((int)bandIndex) + ",";

  json += "\"tx_enabled\":" + String(txEnabled ? "true" : "false") + ",";
  json += "\"tx_every_slot\":" + String(txEverySlot ? "true" : "false") + ",";
  json += "\"si5351_clock\":" + String(si5351Clock) + ",";
#ifdef HAS_NEOPIXEL
  json += "\"led_enabled\":" + String(ledEnabled ? "true" : "false") + ",";
#endif

  json += "\"ntp_server\":\"" + htmlEscape(ntpServer) + "\",";

  json += "\"time_valid\":" + String(tOk ? "true" : "false") + ",";
  json += "\"now_epoch\":" + String((uint32_t)now) + ",";
  json += "\"next_tx_epoch\":" + String((uint32_t)nextTx) + ",";

  json += "\"bands\":[";
  for (size_t i = 0; i < NUM_BANDS; i++) {
    if (i) json += ",";
    json += "{";
    json += "\"name\":\"" + String(BANDS[i].name) + "\",";
    json += "\"dial_hz\":" + String(BANDS[i].dial_hz, 1) + ",";
    json += "\"cal_hz\":" + String(bandCalHz[i], 1) + ",";
    json += "\"clk_out\":" + String(bandClockOut[i]) + ",";
    json += "\"active\":" + String(i == bandIndex ? "true" : "false");
    json += "}";
  }
  json += "]";

  json += "}";
  server.send(200, "application/json", json);
}

void handleScan() {
  int n = WiFi.scanNetworks(false, true);
  String json = "{\"networks\":[";
  for (int i = 0; i < n; i++) {
    if (i) json += ",";
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);
    ssid.replace("\"", "\\\"");
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(rssi) + "}";
  }
  json += "]}";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

void handleSaveWifi() {
  if (!server.hasArg("ssid")) { server.send(400, "text/plain", "Missing ssid"); return; }
  wifiSsid = server.arg("ssid");
  wifiPass = server.hasArg("pass") ? server.arg("pass") : "";
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleSaveNtp() {
  if (!server.hasArg("ntp")) { server.send(400, "text/plain", "Missing ntp"); return; }
  ntpServer = server.arg("ntp");
  ntpServer.trim();
  if (ntpServer.isEmpty()) ntpServer = DEFAULT_NTP_SERVER;
  saveSettings();
  server.send(200, "text/plain", "OK");
}

static bool isValidCallsign(String c) {
  c.trim(); c.toUpperCase();
  if (c.length() < 3 || c.length() > 6) return false;
  for (size_t i = 0; i < c.length(); i++) if (!isalnum((unsigned char)c[i])) return false;
  return true;
}

static bool isValidLocator(String g) {
  g.trim(); g.toUpperCase();
  if (g.length() != 4) return false;
  return (g[0] >= 'A' && g[0] <= 'R' &&
          g[1] >= 'A' && g[1] <= 'R' &&
          isdigit((unsigned char)g[2]) &&
          isdigit((unsigned char)g[3]));
}

void handleSaveWspr() {
  // required fields
  String call = server.arg("call");
  String loc  = server.arg("loc");
  int pwr     = server.arg("pwr").toInt();
  int b       = server.arg("band").toInt();

  bool newTxEn  = server.hasArg("txen") ? (server.arg("txen") == "1") : txEnabled;
  bool newTxAll = server.hasArg("txall") ? (server.arg("txall") == "1") : txEverySlot;

  call.trim(); call.toUpperCase();
  loc.trim();  loc.toUpperCase();

  if (!isValidCallsign(call)) { server.send(400, "text/plain", "Bad callsign"); return; }
  if (!isValidLocator(loc))   { server.send(400, "text/plain", "Bad locator (4 chars)"); return; }
  if (pwr < 0 || pwr > 60)     { server.send(400, "text/plain", "Bad power"); return; }
  if (b < 0 || (size_t)b >= NUM_BANDS) { server.send(400, "text/plain", "Bad band"); return; }

  // parse per-band calibration fields (cal_0..cal_10). If a field is missing, keep current.
  for (size_t i = 0; i < NUM_BANDS; i++) {
    String k = "cal_" + String((int)i);
    if (server.hasArg(k)) {
      bandCalHz[i] = server.arg(k).toDouble();
    }
  }

  // parse per-band clock output fields (clkout_0..clkout_11)
  for (size_t i = 0; i < NUM_BANDS; i++) {
    String k = "clkout_" + String((int)i);
    if (server.hasArg(k)) {
      int clk = server.arg(k).toInt();
      if (clk >= 0 && clk <= 2) {
        bandClockOut[i] = (clk == 0) ? SI5351_CLK0 : (clk == 1) ? SI5351_CLK1 : SI5351_CLK2;
      }
    }
  }

  CALLSIGN  = call;
  LOCATOR   = loc;
  POWER_DBM = (uint8_t)pwr;
  bandIndex = (size_t)b;

  txEnabled   = newTxEn;
  txEverySlot = newTxAll;

  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleSyncTime() {
  bool ok = syncNtpTime();
  server.send(200, "text/plain", ok ? "OK" : "FAIL");
}

void handleSaveClk() {
  if (!server.hasArg("clk")) { server.send(400, "text/plain", "Missing clk"); return; }
  int clk = server.arg("clk").toInt();
  if (clk < 0 || clk > 2) { server.send(400, "text/plain", "Invalid clock"); return; }
  si5351Clock = (clk == 0) ? SI5351_CLK0 : (clk == 1) ? SI5351_CLK1 : SI5351_CLK2;
  saveSettings();
  server.send(200, "text/plain", "OK");
}

#ifdef HAS_NEOPIXEL
void handleSaveLed() {
  bool newLedEn = server.hasArg("leden") ? (server.arg("leden") == "1") : ledEnabled;
  ledEnabled = newLedEn;
  saveSettings();
  if (!ledEnabled) {
    ledOff();
  } else {
    if (isTxActive) {
      ledTx();
    } else {
      ledIdle();
    }
  }
  server.send(200, "text/plain", "OK");
}
#endif

void handleReboot() {
  server.send(200, "text/plain", "Rebooting");
  delay(200);
  ESP.restart();
}

void handleFavicon() {
  server.send(204); // No Content
}

void startWeb() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/scan", handleScan);

  server.on("/save_wifi", HTTP_POST, handleSaveWifi);
  server.on("/save_ntp", HTTP_POST, handleSaveNtp);
  server.on("/save_wspr", HTTP_POST, handleSaveWspr);
  server.on("/save_clk", HTTP_POST, handleSaveClk);
#ifdef HAS_NEOPIXEL
  server.on("/save_led", HTTP_POST, handleSaveLed);
#endif

  server.on("/sync_time", HTTP_POST, handleSyncTime);

  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/favicon.ico", HTTP_GET, handleFavicon);

  server.onNotFound(handleCaptivePortal);

  server.begin();
  Serial.println("Web server started (port 80)");
}

// ---------- WAIT FOR NEXT SLOT ----------
void serviceNetworkWhileWaiting(uint32_t waitMs) {
  uint32_t endMs = millis() + waitMs;
  while ((int32_t)(endMs - millis()) > 0) {
    server.handleClient();
    if (captivePortalActive) dnsServer.processNextRequest();
    delay(5);
  }
}

void waitForNextSlot() {
  time_t now;
  time(&now);

  time_t nextSlot = computeNextTxEpoch(now);
  int waitSec = max(0, (int)(nextSlot - now));

  struct tm tNow, tSlot;
  gmtime_r(&now, &tNow);
  gmtime_r(&nextSlot, &tSlot);

  Serial.printf(
    "UTC now: %02d:%02d:%02d | waiting %d sec\n",
    tNow.tm_hour, tNow.tm_min, tNow.tm_sec, waitSec
  );

  Serial.printf(
    "Next TX slot: %02d:%02d:00 | mode=%s\n\n",
    tSlot.tm_hour, tSlot.tm_min,
    txEverySlot ? "EVERY" : "ALTERNATE"
  );

  ledIdle();
  serviceNetworkWhileWaiting((uint32_t)waitSec * 1000UL);
}

// ---------- SET RF TONE ----------
static inline void setTone(int tone) {
  const double cal = bandCalHz[bandIndex];
  double f = wsprBaseHz() + cal + sessionFreqOffsetHz + (tone * TONE_SPACING_HZ);
  si5351.set_freq((uint64_t)(f * 100ULL), (si5351_clock)bandClockOut[bandIndex]);
}

// ---------- TRANSMIT FRAME ----------
void transmitWSPR() {
  if (!txEnabled) {
    Serial.println("TX disabled — skipping transmit.");
    return;
  }
  if (!timeValid()) {
    Serial.println("Time not valid — skipping transmit.");
    return;
  }

  sessionFreqOffsetHz = random(0, 100);

  const double cal = bandCalHz[bandIndex];
  double carrier = wsprBaseHz() + cal + sessionFreqOffsetHz;

  Serial.printf("Band: %s  Dial: %.4f MHz\n",
                BANDS[bandIndex].name, BANDS[bandIndex].dial_hz / 1e6);
  Serial.printf("Carrier: %.6f MHz  (band cal %+0.1f Hz, scatter %+0.1f Hz)\n",
                carrier / 1e6, cal, sessionFreqOffsetHz);

  Serial.println("Encoding WSPR...");
  jt.wspr_encode(CALLSIGN.c_str(), LOCATOR.c_str(), POWER_DBM, symbols);

  time_t tStart; time(&tStart);
  struct tm ts; gmtime_r(&tStart, &ts);
  Serial.printf("TX START  UTC %02d:%02d:%02d  | expected ~110.6 s\n",
                ts.tm_hour, ts.tm_min, ts.tm_sec);

  rfOn();

  const uint32_t t0ms = millis();
  const uint32_t startUs = micros();

  for (int i = 0; i < 162; i++) {
    setTone(symbols[i]);

    // Target time for end of this symbol
    const uint32_t targetUs = startUs + (uint32_t)(i + 1) * SYMBOL_PERIOD_US;

    // Keep web responsive, but don't extend symbol time beyond target.
    while ((int32_t)(micros() - targetUs) < 0) {
      server.handleClient();
      if (captivePortalActive) dnsServer.processNextRequest();
      delay(1);
    }
  }

  rfOff();

  float elapsed = (millis() - t0ms) / 1000.0f;
  Serial.printf("TX COMPLETE — actual %.2f s\n\n", elapsed);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(800);

#ifdef HAS_NEOPIXEL
  rgb.begin();
  rgb.clear();
  rgb.show();
  ledOff();
#endif

  loadSettings();

  Serial.println("\nESP32 + Si5351 WSPR Beacon (web-configurable)");
  Serial.printf("Callsign %s  Locator %s  Power %u dBm\n",
                CALLSIGN.c_str(), LOCATOR.c_str(), POWER_DBM);
  Serial.printf("Active band: %s\n", BANDS[bandIndex].name);
  Serial.printf("TX enabled: %s  | Slot mode: %s\n",
                txEnabled ? "YES" : "NO",
                txEverySlot ? "EVERY" : "ALTERNATE");
  Serial.printf("NTP server: %s\n", ntpServer.c_str());

  Wire.begin(I2C_SDA, I2C_SCL);

  // Serial.println("Init Si5351...");
  // si5351.init(SI5351_CRYSTAL_LOAD_8PF, SI5351_CRYSTAL, 0);
  // si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  // rfOff();

Serial.println("Init Si5351...");

// HARD mute outputs immediately
si5351.output_enable(SI5351_CLK0, 0);
si5351.output_enable(SI5351_CLK1, 0);
si5351.output_enable(SI5351_CLK2, 0);

// Now initialise the chip
si5351.init(SI5351_CRYSTAL_LOAD_8PF, SI5351_CRYSTAL, 0);

// Optional but recommended: reset PLLs
si5351.pll_reset(SI5351_PLLA);
si5351.pll_reset(SI5351_PLLB);

// Set drive strength for selected clock
si5351.drive_strength((si5351_clock)si5351Clock, SI5351_DRIVE_8MA);

// Ensure frequency is zeroed
si5351.set_freq(0, (si5351_clock)si5351Clock);

// Final safety mute
rfOff();

#ifndef ESP8266
  randomSeed((uint32_t)esp_random());
#else
  randomSeed(micros());
#endif

  // Try STA for 30 seconds, else AP + captive portal
  bool staOk = connectStaWithTimeout(30000);
  if (!staOk) {
    startApModeCaptivePortal();
  }

  // mDNS is most useful on STA
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS started: http://%s.local/\n", HOSTNAME);
  } else {
    Serial.println("mDNS failed to start");
  }

  startWeb();

  // NTP if possible
  if (staOk) {
    syncNtpTime();
  }

  Serial.println("Ready\n");
}

// ---------- LOOP ----------
void loop() {
  // Keep portal responsive all the time
  server.handleClient();
  if (captivePortalActive) dnsServer.processNextRequest();

  // Periodic STA retry if in AP mode and credentials exist
  static uint32_t lastStaTry = 0;
  if (WiFi.status() != WL_CONNECTED && !wifiSsid.isEmpty()) {
    if (millis() - lastStaTry > 180000UL) { // every 3 minutes
      lastStaTry = millis();
      Serial.println("Periodic STA retry...");
      bool ok = connectStaWithTimeout(15000);
      if (ok) {
        syncNtpTime();
      }
    }
  }

  // If time not valid, try NTP periodically when connected
  if (!timeValid()) {
    static uint32_t lastNtpTry = 0;
    if (WiFi.status() == WL_CONNECTED && millis() - lastNtpTry > 30000UL) {
      lastNtpTry = millis();
      syncNtpTime();
    }
    delay(50);
    return;
  }

  waitForNextSlot();
  transmitWSPR();
}
