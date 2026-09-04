// KH7 GPS Field Tester - Makerfabs MaTouch ESP32-S3 SPI TFT 3.5" (ILI9488), v1.2 hardware.
// Connects to the existing KH7 ESP32 WiFi bridge (tools/esp32_s3_uart6_wifi_bridge) over
// TCP at kh7bridge.local:3333, parses the same ASCII telemetry lines the flight controller
// already emits (see Core/Src/telemetry.c), and shows a large, field-readable GPS/NAV
// dashboard. Read-only - never sends anything to the flight controller.
//
// Confirmed pinout (Makerfabs/Project_Touch-Screen-Camera, example/Lovyan_demo/Lovyan_demo.ino) -
// this board uses the classic ESP32 (VSPI), NOT ESP32-S3, despite the similar Makerfabs product line:
//   LCD_MOSI=13 LCD_MISO=12 LCD_SCK=14 LCD_CS=15 LCD_RST=26 LCD_DC=33
// Backlight is not software-controlled on this board revision (no pin_bl in the vendor example).
#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <WiFi.h>
#include <ESPmDNS.h>

static const char *WIFI_SSID = "NachoWi-Fi";
static const char *WIFI_PASSWORD = "NrMaintain1!";
static const char *BRIDGE_HOST = "kh7bridge.local";
static const char *BRIDGE_MDNS_NAME = "kh7bridge"; // without ".local", for MDNS.queryHost()
static const uint16_t BRIDGE_PORT = 3333;
// Fallback fixed IP, tried only if BOTH WiFi.hostByName() and MDNS.queryHost() fail to
// resolve BRIDGE_HOST (2026-08-30) - the bridge sketch
// (tools/esp32_s3_uart6_wifi_bridge/esp32_s3_uart6_wifi_bridge.ino) has never actually
// called MDNS.begin()/MDNS.addService() at any point in this project's history, so
// "kh7bridge.local" has never been resolvable by anything - confirmed the same session
// this was added, when sdlog_analyze.py --host kh7bridge.local also failed and every
// other tool that night had to be run with --host 10.0.0.39 explicitly instead. This
// display had no fallback at all, so it silently retried a lookup that could never
// succeed, forever, with zero telemetry ever reaching the screen. Update this IP if the
// bridge's DHCP lease ever changes; the real fix is adding an mDNS responder to the
// bridge sketch itself so hostname resolution starts working for every tool, not just
// this one - not done here since the bridge board's own availability wasn't confirmed
// when this fallback was added.
static const char *BRIDGE_FALLBACK_IP = "10.0.0.39";

static const uint32_t WIFI_RETRY_MS = 4000;
static const uint32_t BRIDGE_RETRY_MS = 3000;
static const uint32_t BRIDGE_DATA_TIMEOUT_MS = 2000;
static const uint32_t UI_REFRESH_MS = 150; // ~6.7Hz - within the requested 5-10Hz visual refresh band
static const uint32_t STALE_MS = 3000; // no telemetry line of a given kind for this long -> greyed out
static const size_t BRIDGE_READ_CHUNK_BYTES = 256;
static const size_t BRIDGE_POLL_BUDGET_BYTES = 2048;

#define LCD_MOSI 13
#define LCD_MISO 12
#define LCD_SCK  14
#define LCD_CS   15
#define LCD_RST  26
#define LCD_DC   33

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = LCD_SCK;
      cfg.pin_mosi = LCD_MOSI;
      cfg.pin_miso = LCD_MISO;
      cfg.pin_dc = LCD_DC;

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = LCD_CS;
      cfg.pin_rst = LCD_RST;
      cfg.pin_busy = -1;
      cfg.memory_width = 320;
      cfg.memory_height = 480;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

static LGFX lcd;
static WiFiClient tcp;
static bool mdns_started = false;

// ---- Parsed telemetry state (mirrors Core/Src/telemetry.c line formats) ----
// UNCHANGED from the original implementation - this is the parsing/comms/nav "logic"
// half of the file. Only the UI/rendering half below was redesigned.
struct NavTelemetry
{
  // NAV[...]
  unsigned navValid = 0, navRef = 0, navReason = 0, navFix = 0, navSats = 0;
  long navHaccCm = 0;
  unsigned long navAgeMs = 0, navUpdMs = 0, navCv = 0, navCi = 0, navDup = 0, navRej = 0, navDrop = 0;
  uint32_t navMs = 0;

  // NAVPOS[...]
  float posN = 0, posE = 0, velRawN = 0, velRawE = 0, velFiltN = 0, velFiltE = 0;
  uint32_t navposMs = 0;

  // NAVBRK[...]
  unsigned brkReq = 0, brkAct = 0, brkTiltLim = 0, brkAccelLim = 0;
  float brkDesN = 0, brkDesE = 0, brkErrN = 0, brkErrE = 0;
  float brkAccelN = 0, brkAccelE = 0, brkAccelFwd = 0, brkAccelRight = 0;
  float brkAngRoll = 0, brkAngPitch = 0;
  uint32_t navbrkMs = 0;

  // NAVGATE[...]
  unsigned gateNav = 0, gateRef = 0, gateAtt = 0, gateBaro = 0, gateLink = 0, gateLatch = 0;
  uint32_t navgateMs = 0;

  // NAV_LOST[reason=...]
  char lostReason[32] = "-";
  uint32_t lostMs = 0;

  // GPS[...]
  unsigned gpsCfg = 0, gpsHealthy = 0, gpsFix = 0, gpsSats = 0;
  float gpsLat = 0, gpsLon = 0, gpsAlt = 0;
  uint32_t gpsMs = 0;

  // MODE[...]
  char modeName[16] = "-";
  unsigned modeCh6 = 0;
  uint32_t modeMs = 0;

  // ARM[...]
  unsigned armA = 0, armSw = 0;
  uint32_t armMs = 0;

  // ARM_EVENT[DISARM reason=...]
  char lastDisarmReason[24] = "-";
  uint32_t lastDisarmMs = 0;

  // TELARM[ON|OFF] - confirmed state of the armed bench-telemetry toggle.
  unsigned telArmEnabled = 0;
  uint32_t telArmMs = 0;

  // VBAT[...]
  long vbatMv = 0;
  uint32_t vbatMs = 0;

  // BARO[...]
  unsigned baroHealthy = 0;
  long baroCm = 0, baroCmS = 0;
  uint32_t baroMs = 0;
};

static NavTelemetry g_tel;

static char g_lineBuf[224];
static uint8_t g_lineLen = 0;
static uint32_t g_freshNavEvents = 0;
static uint32_t g_uiFrames = 0;
static uint32_t g_navHzX10 = 0;
static uint32_t g_uiHzX10 = 0;
static uint32_t g_lastBridgeByteMs = 0;
// Display-only instrumentation (not nav/control logic): total complete lines received,
// for the bottom diagnostics row's "PKT" counter.
static uint32_t g_totalLines = 0;
static SemaphoreHandle_t g_telMutex = nullptr;
static volatile bool g_wifiConnected = false;
static volatile bool g_bridgeConnected = false;

// Read-only status polling: RC channel 7 (hi=on, lo=off) on the transmitter is the sole
// control for the armed bench telemetry stream now, so this display only queries and
// shows the firmware's confirmed state - it never sends ON/OFF itself. loop() (core 1)
// only queues the query text; bridgeTask (core 0) owns `tcp` and does the actual send,
// so the WiFiClient object is never touched from two cores at once.
static SemaphoreHandle_t g_cmdMutex = nullptr;
static char g_pendingCmd[24] = "";
static volatile bool g_pendingCmdReady = false;

class TelemetryLock
{
public:
  explicit TelemetryLock(SemaphoreHandle_t mutex) : mutex_(mutex)
  {
    xSemaphoreTake(mutex_, portMAX_DELAY);
  }

  ~TelemetryLock()
  {
    xSemaphoreGive(mutex_);
  }

private:
  SemaphoreHandle_t mutex_;
};

static void queueCommand(const char *cmd)
{
  TelemetryLock lock(g_cmdMutex);
  strncpy(g_pendingCmd, cmd, sizeof(g_pendingCmd) - 1);
  g_pendingCmd[sizeof(g_pendingCmd) - 1] = '\0';
  g_pendingCmdReady = true;
}

static void ensureWifi()
{
  static uint32_t lastAttempt = 0;
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }
  uint32_t now = millis();
  if ((now - lastAttempt) < WIFI_RETRY_MS && lastAttempt != 0)
  {
    return;
  }
  lastAttempt = now;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

static bool resolveBridgeIp(IPAddress &ip)
{
  if (WiFi.hostByName(BRIDGE_HOST, ip) && ip != IPAddress(0, 0, 0, 0))
  {
    return true;
  }
  if (!mdns_started)
  {
    mdns_started = MDNS.begin("kh7display");
  }
  if (mdns_started)
  {
    ip = MDNS.queryHost(BRIDGE_MDNS_NAME);
    if (ip != IPAddress(0, 0, 0, 0))
    {
      return true;
    }
  }
  // See BRIDGE_FALLBACK_IP's comment - hostname/mDNS resolution has never actually
  // worked against this bridge, so fall back to the known-good fixed IP rather than
  // retrying a lookup that can never succeed forever with no data ever reaching the
  // screen.
  if (ip.fromString(BRIDGE_FALLBACK_IP))
  {
    Serial.printf("[BRIDGE] hostname/mDNS resolution failed, using fallback IP %s\n", BRIDGE_FALLBACK_IP);
    return true;
  }
  return false;
}

static void ensureBridge()
{
  static uint32_t lastAttempt = 0;
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }
  if (tcp.connected())
  {
    if ((g_lastBridgeByteMs == 0U) || ((millis() - g_lastBridgeByteMs) <= BRIDGE_DATA_TIMEOUT_MS))
    {
      return;
    }
    tcp.stop();
    g_lastBridgeByteMs = 0U;
  }
  uint32_t now = millis();
  if ((now - lastAttempt) < BRIDGE_RETRY_MS && lastAttempt != 0)
  {
    return;
  }
  lastAttempt = now;

  tcp.stop();
  IPAddress ip;
  if (!resolveBridgeIp(ip))
  {
    return;
  }
  if (tcp.connect(ip, BRIDGE_PORT, 1500))
  {
    g_lastBridgeByteMs = millis();
  }
}

// ---- Line parsing (sscanf mirrors the exact printf formats in Core/Src/telemetry.c) ----
// UNCHANGED parsing logic - only the g_totalLines instrumentation bump was added, in
// pollBridge() below, not here.
static void processLine(const char *line)
{
  uint32_t now = millis();
  TelemetryLock lock(g_telMutex);

  if (strncmp(line, "NAV[", 4) == 0 && strncmp(line, "NAVPOS", 6) != 0)
  {
    unsigned v, r, rs, fx, sv;
    long hacc;
    unsigned long age, upd, cv, ci, dup, rej, drop;
    if (sscanf(line,
               "NAV[valid ref reason fix sats hacc_cm age_ms upd_ms cv ci dup rej drop]=[%u %u %u %u %u %ld %lu %lu %lu %lu %lu %lu %lu]",
               &v, &r, &rs, &fx, &sv, &hacc, &age, &upd, &cv, &ci, &dup, &rej, &drop) == 13)
    {
      if (cv != g_tel.navCv)
      {
        g_freshNavEvents++;
      }
      g_tel.navValid = v; g_tel.navRef = r; g_tel.navReason = rs; g_tel.navFix = fx; g_tel.navSats = sv;
      g_tel.navHaccCm = hacc; g_tel.navAgeMs = age; g_tel.navUpdMs = upd;
      g_tel.navCv = cv; g_tel.navCi = ci; g_tel.navDup = dup; g_tel.navRej = rej; g_tel.navDrop = drop;
      g_tel.navMs = now;
    }
    return;
  }

  if (strncmp(line, "NAVPOS[", 7) == 0)
  {
    float n, e, vrn, vre, vfn, vfe;
    if (sscanf(line, "NAVPOS[n e]=[%f %f] velraw=[%f %f] velfilt=[%f %f]", &n, &e, &vrn, &vre, &vfn, &vfe) == 6)
    {
      g_tel.posN = n; g_tel.posE = e;
      g_tel.velRawN = vrn; g_tel.velRawE = vre;
      g_tel.velFiltN = vfn; g_tel.velFiltE = vfe;
      g_tel.navposMs = now;
    }
    return;
  }

  if (strncmp(line, "NAVBRK[", 7) == 0)
  {
    unsigned req, act, tl, al;
    float dn, de, en, ee, an, ae, afwd, aright, ar, ap;
    if (sscanf(line,
               "NAVBRK[req act tiltlim acclim]=[%u %u %u %u] desvel=[%f %f] velerr=[%f %f] accel=[%f %f %f %f] ang=[%f %f]",
               &req, &act, &tl, &al, &dn, &de, &en, &ee, &an, &ae, &afwd, &aright, &ar, &ap) == 14)
    {
      g_tel.brkReq = req; g_tel.brkAct = act; g_tel.brkTiltLim = tl; g_tel.brkAccelLim = al;
      g_tel.brkDesN = dn; g_tel.brkDesE = de; g_tel.brkErrN = en; g_tel.brkErrE = ee;
      g_tel.brkAccelN = an; g_tel.brkAccelE = ae; g_tel.brkAccelFwd = afwd; g_tel.brkAccelRight = aright;
      g_tel.brkAngRoll = ar; g_tel.brkAngPitch = ap;
      g_tel.navbrkMs = now;
    }
    return;
  }

  if (strncmp(line, "NAV_LOST[", 9) == 0)
  {
    char reason[32];
    if (sscanf(line, "NAV_LOST[reason=%31[^]]]", reason) == 1)
    {
      strncpy(g_tel.lostReason, reason, sizeof(g_tel.lostReason) - 1);
      g_tel.lostReason[sizeof(g_tel.lostReason) - 1] = '\0';
      g_tel.lostMs = now;
    }
    return;
  }

  if (strncmp(line, "NAVGATE[", 8) == 0)
  {
    unsigned nav, ref, att, baro, link, latch;
    if (sscanf(line, "NAVGATE[nav ref att baro link latch]=[%u %u %u %u %u %u]",
               &nav, &ref, &att, &baro, &link, &latch) == 6)
    {
      g_tel.gateNav = nav; g_tel.gateRef = ref; g_tel.gateAtt = att;
      g_tel.gateBaro = baro; g_tel.gateLink = link; g_tel.gateLatch = latch;
      g_tel.navgateMs = now;
    }
    return;
  }

  if (strncmp(line, "GPS[", 4) == 0)
  {
    unsigned cfg, healthy, fix, sats;
    float lat, lon, alt;
    if (sscanf(line, "GPS[cfg healthy fix sats]=[%u %u %u %u] lla=[%f %f %f]",
               &cfg, &healthy, &fix, &sats, &lat, &lon, &alt) == 7)
    {
      g_tel.gpsCfg = cfg; g_tel.gpsHealthy = healthy; g_tel.gpsFix = fix; g_tel.gpsSats = sats;
      g_tel.gpsLat = lat; g_tel.gpsLon = lon; g_tel.gpsAlt = alt;
      g_tel.gpsMs = now;
    }
    return;
  }

  if (strncmp(line, "MODE[", 5) == 0)
  {
    char name[16];
    unsigned ch6;
    if (sscanf(line, "MODE[name=%15[^ ] ch6=%u]", name, &ch6) == 2)
    {
      strncpy(g_tel.modeName, name, sizeof(g_tel.modeName) - 1);
      g_tel.modeName[sizeof(g_tel.modeName) - 1] = '\0';
      g_tel.modeCh6 = ch6;
      g_tel.modeMs = now;
    }
    return;
  }

  if (strncmp(line, "ARM[", 4) == 0)
  {
    unsigned a, sw, lowSeen, thr, s1, s2, s3, s4;
    if (sscanf(line, "ARM[a=%u sw=%u lowSeen=%u thr=%u m]=[%u %u %u %u]",
               &a, &sw, &lowSeen, &thr, &s1, &s2, &s3, &s4) == 8)
    {
      g_tel.armA = a; g_tel.armSw = sw;
      g_tel.armMs = now;
    }
    return;
  }

  if (strncmp(line, "ARM_EVENT[DISARM", 17) == 0)
  {
    char reason[24];
    unsigned link, armUs;
    unsigned long ageMs;
    if (sscanf(line, "ARM_EVENT[DISARM reason=%23[^ ] link=%u arm_us=%u age_ms=%lu]",
               reason, &link, &armUs, &ageMs) == 4)
    {
      // Fires immediately after the controller stops motors, before any SD flush -
      // reflect disarmed state right away instead of waiting for the next slower ARM[...] line.
      g_tel.armA = 0;
      g_tel.armMs = now;
      strncpy(g_tel.lastDisarmReason, reason, sizeof(g_tel.lastDisarmReason) - 1);
      g_tel.lastDisarmReason[sizeof(g_tel.lastDisarmReason) - 1] = '\0';
      g_tel.lastDisarmMs = now;
    }
    return;
  }

  if (strncmp(line, "VBAT[", 5) == 0)
  {
    long mv;
    unsigned long raw;
    if (sscanf(line, "VBAT[mV raw]=[%ld %lu]", &mv, &raw) == 2)
    {
      g_tel.vbatMv = mv;
      g_tel.vbatMs = now;
    }
    return;
  }

  if (strncmp(line, "BARO[", 5) == 0)
  {
    unsigned healthy;
    long cm, cms;
    if (sscanf(line, "BARO[healthy cm cm_s]=[%u %ld %ld]", &healthy, &cm, &cms) == 3)
    {
      g_tel.baroHealthy = healthy; g_tel.baroCm = cm; g_tel.baroCmS = cms;
      g_tel.baroMs = now;
    }
    return;
  }

  if (strncmp(line, "TELARM[", 7) == 0)
  {
    if (strncmp(line, "TELARM[ON]", 10) == 0)
    {
      g_tel.telArmEnabled = 1;
      g_tel.telArmMs = now;
    }
    else if (strncmp(line, "TELARM[OFF]", 11) == 0)
    {
      g_tel.telArmEnabled = 0;
      g_tel.telArmMs = now;
    }
    return;
  }
}

static void pollBridge()
{
  uint8_t buffer[BRIDGE_READ_CHUNK_BYTES];
  size_t processed = 0;

  while (tcp.connected() && tcp.available() > 0 && processed < BRIDGE_POLL_BUDGET_BYTES)
  {
    int available = tcp.available();
    size_t remaining = BRIDGE_POLL_BUDGET_BYTES - processed;
    size_t toRead = (size_t)available;
    if (toRead > sizeof(buffer))
    {
      toRead = sizeof(buffer);
    }
    if (toRead > remaining)
    {
      toRead = remaining;
    }

    int count = tcp.read(buffer, toRead);
    if (count <= 0)
    {
      break;
    }
    g_lastBridgeByteMs = millis();
    processed += (size_t)count;

    for (int i = 0; i < count; i++)
    {
      char c = (char)buffer[i];
      if (c == '\n')
      {
        g_lineBuf[g_lineLen] = '\0';
        if (g_lineLen > 0)
        {
          processLine(g_lineBuf);
          g_totalLines++;
        }
        g_lineLen = 0;
      }
      else if (c != '\r')
      {
        if (g_lineLen < (sizeof(g_lineBuf) - 1))
        {
          g_lineBuf[g_lineLen++] = c;
        }
        else
        {
          g_lineLen = 0; // overflow guard: drop the oversized line
        }
      }
    }
  }
}

static void bridgeTask(void *parameter)
{
  (void)parameter;

  for (;;)
  {
    ensureWifi();
    ensureBridge();
    pollBridge();

    if (g_pendingCmdReady && tcp.connected())
    {
      char cmd[sizeof(g_pendingCmd)];
      {
        TelemetryLock lock(g_cmdMutex);
        strncpy(cmd, g_pendingCmd, sizeof(cmd));
        g_pendingCmdReady = false;
      }
      tcp.print(cmd);
      tcp.print("\n");
    }

    g_wifiConnected = (WiFi.status() == WL_CONNECTED);
    g_bridgeConnected = tcp.connected();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =====================================================================================
// ---- UI (redesigned 2026-08-16: sprite-based field display, replaces the old
// direct-fillRect-per-field renderer; switched to portrait 2026-08-16) ----
//
// Static content (borders, section headers, field labels, units) is drawn ONCE to the
// physical panel by drawStaticLayout(). Every value that changes at telemetry rate lives
// in a small LGFX_Sprite (LovyanGFX's off-screen canvas - this project uses LovyanGFX,
// NOT TFT_eSPI, so TFT_eSprite doesn't apply here) sized to just its own region and
// pushed to a FIXED rectangle that never overlaps a static label's rectangle - unlike the
// old fillRect-per-field approach, a pushSprite() can't bleed into an adjacent label
// regardless of how the value's string length changes between frames, so the old
// "erased part of the neighboring label" failure mode this file used to guard against
// with a comment is structurally not possible here.
//
// This board is a CLASSIC ESP32 (see the pinout comment at the top of this file) with no
// PSRAM, so every sprite here uses 4-bit (16-color) depth: the whole UI only ever needs
// ~6 flat colors (black bg, white, cyan, green, red, yellow), well within a 16-entry
// palette, and this cuts sprite RAM to a quarter of naive 16-bit color - see the RAM
// budget in initDisplaySprites().
//
// Portrait note: the panel is natively 320x480 (see LGFX::_panel_instance.config() above -
// memory/panel width=320, height=480), so portrait just needs setRotation(0) in setup()
// instead of the old setRotation(1) landscape rotation - no panel config changes needed.
// =====================================================================================

#define COLOR_BG TFT_BLACK
#define COLOR_LABEL 0x7BEF // light grey
#define COLOR_HEADER TFT_CYAN
#define COLOR_GOOD TFT_GREEN
#define COLOR_BAD TFT_RED
#define COLOR_WARN TFT_YELLOW
#define COLOR_STALE 0x4208 // dark grey, for values that haven't updated recently
#define COLOR_TEXT TFT_WHITE
#define COLOR_DATA_LABEL TFT_ORANGE // static field labels across all data panels (GPS, Position, Velocity, NAV BRAKE)

// Mirrors Core/Src/app.c's APP_NAV_BRAKE_MAX_VEL_MPS - NOT telemetered by the firmware,
// so this is a static display-only reference value, not a live/recomputed limit. Update
// this if that firmware constant changes.
#define NAVBRAKE_MAX_VEL_MPS_DISPLAY_ONLY 1.5f

// Set once verified against real hardware (see the "no gate values showing" debug pass) -
// leave on for now since it's cheap and useful; harmless to leave enabled permanently.
#define GATE_DEBUG_SERIAL 1

static const char *fixTypeName(unsigned fix)
{
  switch (fix)
  {
    case 0: return "No fix";
    case 1: return "Dead reck.";
    case 2: return "2D";
    case 3: return "3D";
    case 4: return "GNSS+DR";
    case 5: return "Time only";
    default: return "?";
  }
}

static const char *NAV_REASON_NAMES[] = {
    "NONE", "NO_3D_FIX", "GPS_STALE", "BAD_UPDATE_INTERVAL", "POOR_ACCURACY",
    "VELOCITY_INVALID", "POSITION_JUMP", "VELOCITY_JUMP", "REACQUIRING",
    "NO_REFERENCE", "NONFINITE"};

static const char *reasonName(unsigned r)
{
  if (r < (sizeof(NAV_REASON_NAMES) / sizeof(NAV_REASON_NAMES[0])))
  {
    return NAV_REASON_NAMES[r];
  }
  return "?";
}

static uint16_t staleColor(uint32_t lastMs, uint16_t freshColor)
{
  if (lastMs == 0 || (millis() - lastMs) > STALE_MS)
  {
    return COLOR_STALE;
  }
  return freshColor;
}

// ---- Layout (portrait 320x480 after setRotation(0)) ----
// Row y-coordinates, top to bottom. Position and Velocity are stacked vertically here
// (they were side-by-side in the old landscape layout) since 320px is too narrow for
// two side-by-side panels with room to breathe.
#define ROW_TITLE_Y      2
#define HLINE1_Y         22
#define ROW_GPS_Y        28   // GPS status block start (2 lines)
#define HLINE2_Y         76
#define POS_TITLE_Y      82
#define POS_LABEL_X      6
#define POS_VALUE_X      60
#define POS_ROW0_Y       (POS_TITLE_Y + 22) // N
#define POS_ROW1_Y       (POS_TITLE_Y + 44) // E
#define POS_ROW2_Y       (POS_TITLE_Y + 66) // ALT
#define VEL_TITLE_Y      178
#define VEL_LABEL_X      6
#define VEL_VALUE_X      90
#define VEL_ROW0_Y       (VEL_TITLE_Y + 22) // RAW N
#define VEL_ROW1_Y       (VEL_TITLE_Y + 42) // RAW E
#define VEL_ROW2_Y       (VEL_TITLE_Y + 62) // FILT N
#define VEL_ROW3_Y       (VEL_TITLE_Y + 82) // FILT E
#define HLINE3_Y         284
#define NAVBRK_TITLE_Y   290
#define NAVBRK_ROW1_Y    310  // Requested: / Active:
#define NAVBRK_GATE_Y    332  // Gates: ...
#define NAVBRK_ROW3_Y    354  // Speed: / Threshold:
#define HLINE4_Y         378
#define DIAG_ROW1_Y      384  // PKT / AGE / ERR
#define DIAG_ROW2_Y      406  // GPS UART RX Hz / DROP
#define SCREEN_W         320

// ---- Sprites (created once in initDisplaySprites(), reused for the life of the app) ----
static LGFX_Sprite spriteHz(&lcd);          // top-right Hz readout
static LGFX_Sprite spriteGpsStatus(&lcd);   // 2 lines: GPS fix/sat/hAcc + UART/NAV status
static LGFX_Sprite spritePosValues(&lcd);   // 3 lines: N / E / ALT (numbers only)
static LGFX_Sprite spriteVelValues(&lcd);   // 4 lines: raw N/E, filt N/E (numbers only)
static LGFX_Sprite spriteReqActive(&lcd);   // "YES/NO" x2 on the Requested/Active row
static LGFX_Sprite spriteGates(&lcd);       // the whole "Gates: ..." row (own change-detect)
static LGFX_Sprite spriteSpeedThresh(&lcd); // Speed/Threshold numeric values
static LGFX_Sprite spriteDiag(&lcd);        // 2 lines: bottom diagnostics
static LGFX_Sprite spriteModeTitle(&lcd);   // dynamic mode name, replaces the old static "NAV BRAKE" title

static bool g_spriteOk[9] = {false, false, false, false, false, false, false, false, false};
enum SpriteIdx
{
  SPR_HZ = 0, SPR_GPS_STATUS, SPR_POS, SPR_VEL, SPR_REQ_ACTIVE, SPR_GATES, SPR_SPEED_THRESH, SPR_DIAG,
  SPR_MODE_TITLE
};

// Change-detection caches for the sprites the spec asks to only redraw on change
// (GPS fix/sat, gate flags, NAV BRAKE Requested/Active). Deliberately simple string
// compares, same pattern the old setField() used - "do not make this unnecessarily
// complicated" per the redesign brief.
static char g_cacheGpsStatus[64] = "";
static char g_cacheGates[48] = "";
static char g_cacheReqActive[24] = "";
static char g_cacheModeTitle[24] = "";

static bool createSpriteChecked(LGFX_Sprite &spr, int16_t w, int16_t h, bool *okFlag, const char *name)
{
  // 16-bit RGB565 - true color, no palette. Switched from 4-bit palette mode
  // (2026-08-16) after COLOR_GOOD (green) rendered as invisible black-on-black
  // specifically on the gates row and on the Position panel once a GPS fix was
  // attained - both are exactly the two places that switch to COLOR_GOOD - consistent
  // with that one color failing to resolve correctly in the 16-entry palette. 16bpp
  // costs ~4x the RAM but eliminates the whole class of palette-mapping bugs outright;
  // per spec, status-color correctness takes priority over the memory savings.
  spr.setColorDepth(16);
  void *buf = spr.createSprite(w, h);
  *okFlag = (buf != nullptr);
  if (!*okFlag)
  {
    // MEMORY SAFETY: report, don't crash. That panel's updateXSprite() below checks
    // g_spriteOk[] and simply skips drawing (leaves that region blank) rather than
    // dereferencing a failed allocation.
    Serial.printf("[DISPLAY] WARNING: sprite '%s' (%dx%d, 16bpp) failed to allocate - that panel will stay blank\n",
                  name, (int)w, (int)h);
  }
  return *okFlag;
}

static void initDisplaySprites()
{
  // Approximate RAM budget at 16bpp (2 bytes/px), portrait sprites (trimmed to fit
  // actual content, not left at the earlier 4bpp-era margins): Hz ~3.2KB,
  // GpsStatus ~27KB, Pos ~23.4KB, Vel ~29.7KB, ReqActive ~8.4KB, Gates ~10.5KB,
  // SpeedThresh ~8.4KB, Diag ~29.5KB - total ~140KB. A classic ESP32 has 520KB SRAM;
  // with WiFi active (no BT) this leaves comfortable headroom.
  createSpriteChecked(spriteHz, 90, 18, &g_spriteOk[SPR_HZ], "hz");
  createSpriteChecked(spriteGpsStatus, 312, 44, &g_spriteOk[SPR_GPS_STATUS], "gpsStatus");
  createSpriteChecked(spritePosValues, 200, 60, &g_spriteOk[SPR_POS], "posValues");
  createSpriteChecked(spriteVelValues, 190, 80, &g_spriteOk[SPR_VEL], "velValues");
  createSpriteChecked(spriteReqActive, 240, 18, &g_spriteOk[SPR_REQ_ACTIVE], "reqActive");
  createSpriteChecked(spriteGates, 300, 18, &g_spriteOk[SPR_GATES], "gates");
  createSpriteChecked(spriteSpeedThresh, 240, 18, &g_spriteOk[SPR_SPEED_THRESH], "speedThresh");
  createSpriteChecked(spriteDiag, 260, 58, &g_spriteOk[SPR_DIAG], "diag"); // 3 lines incl. aircraft VBAT
  createSpriteChecked(spriteModeTitle, 160, 20, &g_spriteOk[SPR_MODE_TITLE], "modeTitle");
}

// ---- Static layout: borders, headers, field labels. Drawn once, never redrawn per
// telemetry update (per the "no full-screen refreshes on every update" requirement). ----
static void drawStaticLayout()
{
  lcd.fillScreen(COLOR_BG);

  lcd.setTextColor(COLOR_HEADER, COLOR_BG);
  lcd.setTextSize(2);
  lcd.setCursor(6, ROW_TITLE_Y);
  lcd.print("K7H FIELD TESTER");

  lcd.drawFastHLine(0, HLINE1_Y, SCREEN_W, COLOR_LABEL);
  lcd.drawFastHLine(0, HLINE2_Y, SCREEN_W, COLOR_LABEL);
  lcd.drawFastHLine(0, HLINE3_Y, SCREEN_W, COLOR_LABEL);
  lcd.drawFastHLine(0, HLINE4_Y, SCREEN_W, COLOR_LABEL);

  // POSITION panel (static labels) - orange: this whole panel is raw GPS data (lat/lon/alt).
  lcd.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  lcd.setTextSize(1);
  lcd.setCursor(POS_LABEL_X, POS_TITLE_Y);
  lcd.print("POSITION");
  lcd.setCursor(POS_LABEL_X, POS_ROW0_Y);
  lcd.print("N");
  lcd.setCursor(POS_LABEL_X, POS_ROW1_Y);
  lcd.print("E");
  lcd.setCursor(POS_LABEL_X, POS_ROW2_Y);
  lcd.print("ALT");

  // VELOCITY panel (static labels)
  lcd.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  lcd.setCursor(VEL_LABEL_X, VEL_TITLE_Y);
  lcd.print("VELOCITY");
  lcd.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  lcd.setCursor(VEL_LABEL_X, VEL_ROW0_Y);
  lcd.print("RAW N");
  lcd.setCursor(VEL_LABEL_X, VEL_ROW1_Y);
  lcd.print("RAW E");
  lcd.setCursor(VEL_LABEL_X, VEL_ROW2_Y);
  lcd.print("FILT N");
  lcd.setCursor(VEL_LABEL_X, VEL_ROW3_Y);
  lcd.print("FILT E");

  // NAV BRAKE panel - the title itself is now dynamic (see updateModeTitleSprite()),
  // showing the flight controller's actual current mode (MODE[...] telemetry) rather
  // than a fixed "NAV BRAKE" label, since this whole panel's data is only live while
  // that mode is actually selected - static labels below only.
  lcd.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  lcd.setCursor(6, NAVBRK_ROW1_Y);
  lcd.print("Requested:");
  lcd.setCursor(180, NAVBRK_ROW1_Y);
  lcd.print("Active:");
  lcd.setCursor(6, NAVBRK_GATE_Y);
  lcd.print("Gates:");
  lcd.setCursor(6, NAVBRK_ROW3_Y);
  lcd.print("Speed:");
  lcd.setCursor(6, NAVBRK_ROW3_Y + 18);
  lcd.print("Threshold:");
}

// ---- Dynamic region updates: each fills its sprite off-screen, then pushes it in one
// operation (per the "clear -> draw -> push" pattern requested). ----

static void updateHzSprite(uint32_t uiHzX10)
{
  if (!g_spriteOk[SPR_HZ]) return;
  spriteHz.fillSprite(COLOR_BG);
  spriteHz.setTextColor(COLOR_TEXT, COLOR_BG);
  spriteHz.setTextSize(2);
  char buf[24];
  snprintf(buf, sizeof(buf), "%lu.%luHz", (unsigned long)(uiHzX10 / 10U), (unsigned long)(uiHzX10 % 10U));
  spriteHz.setTextDatum(lgfx::top_right);
  spriteHz.drawString(buf, spriteHz.width() - 2, 1);
  spriteHz.pushSprite(SCREEN_W - spriteHz.width() - 4, ROW_TITLE_Y);
}

// True once a 3D (or better - GNSS+DR) fix is attained and the sample is still fresh -
// drives the "all GPS values go green on 3D fix" request across the status row and the
// Position panel (both are genuinely GPS-derived values).
static bool gps3dFixGood(const NavTelemetry &tel, bool navFresh)
{
  return navFresh && (tel.navFix >= 3);
}

static void updateGpsStatusSprite(const NavTelemetry &tel, bool wifiOk, bool bridgeOk)
{
  if (!g_spriteOk[SPR_GPS_STATUS]) return;

  bool navFresh = (tel.navMs != 0) && ((millis() - tel.navMs) < STALE_MS);
  bool fixGood = gps3dFixGood(tel, navFresh);
  uint16_t fixColor = !navFresh ? COLOR_STALE : (fixGood ? COLOR_GOOD : (tel.navFix > 0 ? COLOR_WARN : COLOR_BAD));
  uint16_t linkColor = (wifiOk && bridgeOk) ? COLOR_GOOD : COLOR_BAD;
  uint16_t navActiveColor = !navFresh ? COLOR_STALE : (tel.navValid ? COLOR_GOOD : COLOR_BAD);
  // The GPS status line's text itself (not just the dot) turns green once a 3D+ fix is
  // attained, per request - falls back to plain white/stale otherwise.
  uint16_t gpsTextColor = fixGood ? COLOR_GOOD : staleColor(tel.navMs, COLOR_TEXT);

  char cacheKey[64];
  snprintf(cacheKey, sizeof(cacheKey), "%u|%u|%ld|%d|%d|%u", tel.navFix, tel.navSats, tel.navHaccCm,
           wifiOk && bridgeOk, tel.navValid, navFresh);
  if (strncmp(cacheKey, g_cacheGpsStatus, sizeof(cacheKey)) == 0)
  {
    return; // GPS fix/sat and link/nav state unchanged - skip the redraw (per spec)
  }
  strncpy(g_cacheGpsStatus, cacheKey, sizeof(g_cacheGpsStatus) - 1);
  g_cacheGpsStatus[sizeof(g_cacheGpsStatus) - 1] = '\0';

  spriteGpsStatus.fillSprite(COLOR_BG);
  spriteGpsStatus.setTextSize(1);
  spriteGpsStatus.setTextDatum(lgfx::top_left);

  // Fixed pixel slots (labels orange, values follow gpsTextColor) - same
  // known-working pattern as the gates row, not accumulated textWidth() sums.
  spriteGpsStatus.fillCircle(4, 5, 4, fixColor);
  char buf[24];
  spriteGpsStatus.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  spriteGpsStatus.drawString("GPS", 14, 0);
  spriteGpsStatus.setTextColor(gpsTextColor, COLOR_BG);
  spriteGpsStatus.drawString(fixTypeName(tel.navFix), 38, 0);
  spriteGpsStatus.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  spriteGpsStatus.drawString("SAT", 98, 0);
  spriteGpsStatus.setTextColor(gpsTextColor, COLOR_BG);
  snprintf(buf, sizeof(buf), "%u", tel.navSats);
  spriteGpsStatus.drawString(buf, 122, 0);
  spriteGpsStatus.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
  spriteGpsStatus.drawString("hAcc", 142, 0);
  spriteGpsStatus.setTextColor(gpsTextColor, COLOR_BG);
  snprintf(buf, sizeof(buf), "%.1fm", tel.navHaccCm / 100.0f);
  spriteGpsStatus.drawString(buf, 172, 0);

  spriteGpsStatus.fillCircle(4, 27, 4, linkColor);
  spriteGpsStatus.setTextColor(COLOR_TEXT, COLOR_BG);
  spriteGpsStatus.setCursor(14, 22);
  spriteGpsStatus.print(wifiOk && bridgeOk ? "UART OK" : "UART --");

  spriteGpsStatus.fillCircle(160, 27, 4, navActiveColor);
  spriteGpsStatus.setCursor(170, 22);
  spriteGpsStatus.print(navFresh ? (tel.navValid ? "NAV ACTIVE" : "NAV INVALID") : "NAV WAIT");

  spriteGpsStatus.pushSprite(4, ROW_GPS_Y);
}

static void updatePosVelSprites(const NavTelemetry &tel)
{
  bool navFresh = (tel.navMs != 0) && ((millis() - tel.navMs) < STALE_MS);
  bool fixGood = gps3dFixGood(tel, navFresh);

  // High-rate numeric telemetry - update every refresh cycle, no change-gating
  // (per spec: "velocity may still update at the normal display rate").
  if (g_spriteOk[SPR_POS])
  {
    spritePosValues.fillSprite(COLOR_BG);
    spritePosValues.setTextSize(1);
    // POSITION is raw GPS lat/lon/alt - genuinely "GPS values", so it follows the same
    // green-on-3D-fix rule as the status row above.
    uint16_t c = fixGood ? COLOR_GOOD : staleColor(tel.navposMs, COLOR_TEXT);
    spritePosValues.setTextColor(c, COLOR_BG);
    spritePosValues.setTextDatum(lgfx::top_right);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.6f", tel.gpsLat);
    spritePosValues.drawString(buf, spritePosValues.width(), 0);
    snprintf(buf, sizeof(buf), "%.6f", tel.gpsLon);
    spritePosValues.drawString(buf, spritePosValues.width(), 20);
    snprintf(buf, sizeof(buf), "%.1f m", tel.gpsAlt);
    spritePosValues.drawString(buf, spritePosValues.width(), 40);
    spritePosValues.pushSprite(POS_VALUE_X, POS_ROW0_Y - 2);
  }

  if (g_spriteOk[SPR_VEL])
  {
    spriteVelValues.fillSprite(COLOR_BG);
    spriteVelValues.setTextSize(1);
    uint16_t c = staleColor(tel.navposMs, COLOR_TEXT);
    spriteVelValues.setTextColor(c, COLOR_BG);
    spriteVelValues.setTextDatum(lgfx::top_right);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.2f m/s", tel.velRawN);
    spriteVelValues.drawString(buf, spriteVelValues.width(), 0);
    snprintf(buf, sizeof(buf), "%.2f m/s", tel.velRawE);
    spriteVelValues.drawString(buf, spriteVelValues.width(), 20);
    snprintf(buf, sizeof(buf), "%.2f m/s", tel.velFiltN);
    spriteVelValues.drawString(buf, spriteVelValues.width(), 40);
    snprintf(buf, sizeof(buf), "%.2f m/s", tel.velFiltE);
    spriteVelValues.drawString(buf, spriteVelValues.width(), 60);
    spriteVelValues.pushSprite(VEL_VALUE_X, VEL_ROW0_Y - 2);
  }
}

static void updateModeTitleSprite(const NavTelemetry &tel)
{
  if (!g_spriteOk[SPR_MODE_TITLE]) return;

  bool modeFresh = (tel.modeMs != 0) && ((millis() - tel.modeMs) < STALE_MS);
  char cacheKey[24];
  snprintf(cacheKey, sizeof(cacheKey), "%s|%d", tel.modeName, modeFresh);
  if (strncmp(cacheKey, g_cacheModeTitle, sizeof(cacheKey)) == 0)
  {
    return; // mode name unchanged - skip the redraw
  }
  strncpy(g_cacheModeTitle, cacheKey, sizeof(g_cacheModeTitle) - 1);
  g_cacheModeTitle[sizeof(g_cacheModeTitle) - 1] = '\0';

  // Green specifically in NAVBRAKE - that's the one mode where this whole panel's data
  // (Requested/Active/Gates/Speed) is actually live; any other mode it's necessarily
  // stale/inactive by firmware design (GPS/Nav_Update() only run in NAVBRAKE), so this
  // is the one place that distinction needs to be obvious at a glance.
  bool isNavBrake = modeFresh && (strncmp(tel.modeName, "NAVBRAKE", sizeof(tel.modeName)) == 0);
  uint16_t c = !modeFresh ? COLOR_STALE : (isNavBrake ? COLOR_GOOD : COLOR_HEADER);

  spriteModeTitle.fillSprite(COLOR_BG);
  spriteModeTitle.setTextSize(1);
  spriteModeTitle.setTextColor(c, COLOR_BG);
  spriteModeTitle.setCursor(0, 0);
  spriteModeTitle.print(modeFresh ? tel.modeName : "MODE ?");

  spriteModeTitle.pushSprite(6, NAVBRK_TITLE_Y);
}

static void updateReqActiveSprite(const NavTelemetry &tel)
{
  if (!g_spriteOk[SPR_REQ_ACTIVE]) return;

  bool brkFresh = (tel.navbrkMs != 0) && ((millis() - tel.navbrkMs) < STALE_MS);
  char cacheKey[24];
  snprintf(cacheKey, sizeof(cacheKey), "%u|%u|%d", tel.brkReq, tel.brkAct, brkFresh);
  if (strncmp(cacheKey, g_cacheReqActive, sizeof(cacheKey)) == 0)
  {
    return; // NAV BRAKE Requested/Active unchanged - skip (per spec)
  }
  strncpy(g_cacheReqActive, cacheKey, sizeof(g_cacheReqActive) - 1);
  g_cacheReqActive[sizeof(g_cacheReqActive) - 1] = '\0';

  spriteReqActive.fillSprite(COLOR_BG);
  spriteReqActive.setTextSize(1);

  uint16_t reqColor = !brkFresh ? COLOR_STALE : (tel.brkReq ? COLOR_GOOD : COLOR_TEXT);
  uint16_t actColor = !brkFresh ? COLOR_STALE : (tel.brkAct ? COLOR_GOOD : COLOR_TEXT);
  spriteReqActive.setTextColor(reqColor, COLOR_BG);
  spriteReqActive.setCursor(0, 0);
  spriteReqActive.print(tel.brkReq ? "YES" : "NO ");

  spriteReqActive.setTextColor(actColor, COLOR_BG);
  spriteReqActive.setCursor(174, 0); // aligns after the static "Active:" label at x=180
  spriteReqActive.print(tel.brkAct ? "YES" : "NO ");

  spriteReqActive.pushSprite(70, NAVBRK_ROW1_Y); // aligns after the static "Requested:" label
}

static void updateGateSprite(const NavTelemetry &tel)
{
  if (!g_spriteOk[SPR_GATES]) return;

  bool gateFresh = (tel.navgateMs != 0) && ((millis() - tel.navgateMs) < STALE_MS);
  char cacheKey[16];
  snprintf(cacheKey, sizeof(cacheKey), "%u%u%u%u%u%d", tel.gateNav, tel.gateRef, tel.gateAtt,
           tel.gateBaro, tel.gateLink, gateFresh);
  if (strncmp(cacheKey, g_cacheGates, sizeof(cacheKey)) == 0)
  {
    return; // gate flags unchanged - skip the redraw (per spec)
  }
  strncpy(g_cacheGates, cacheKey, sizeof(g_cacheGates) - 1);
  g_cacheGates[sizeof(g_cacheGates) - 1] = '\0';

#if GATE_DEBUG_SERIAL
  Serial.printf("[GATES] fresh=%d nav=%u ref=%u att=%u baro=%u link=%u (navgateMs age=%lums)\n",
                gateFresh, tel.gateNav, tel.gateRef, tel.gateAtt, tel.gateBaro, tel.gateLink,
                tel.navgateMs == 0 ? 0UL : (unsigned long)(millis() - tel.navgateMs));
#endif

  spriteGates.fillSprite(COLOR_BG);
  spriteGates.setTextSize(1);

  // Slash formatting, real gate booleans only - never recomputed here.
  struct GateItem { const char *label; unsigned value; };
  const GateItem gates[5] = {
      {"Nav", tel.gateNav}, {"Ref", tel.gateRef}, {"Att", tel.gateAtt},
      {"Baro", tel.gateBaro}, {"Link", tel.gateLink}};

  // Fixed pixel slots (not accumulated textWidth() sums) - one drawString() per gate for
  // the label, one for "/value" combined. This is the same drawString()-at-a-known-offset
  // pattern already working in updatePosVelSprites()/updateHzSprite(), deliberately
  // avoiding the multi-segment setCursor()+print()+textWidth() accumulation this row used
  // before, which was the one thing structurally different about this sprite vs. every
  // other one in this file.
  // Slot pitch tightened from 60 to 48px (~2 chars less gap) per request, label-to-value
  // offset trimmed from 34 to 28 to match.
  static const int16_t GATE_SLOT_X[5] = {0, 48, 96, 144, 192};
  spriteGates.setTextDatum(lgfx::top_left);
  for (int i = 0; i < 5; i++)
  {
    uint16_t valColor = !gateFresh ? COLOR_STALE : (gates[i].value ? COLOR_GOOD : COLOR_BAD);
    char valBuf[4];
    snprintf(valBuf, sizeof(valBuf), "/%u", gates[i].value ? 1U : 0U);

    spriteGates.setTextColor(COLOR_DATA_LABEL, COLOR_BG);
    spriteGates.drawString(gates[i].label, GATE_SLOT_X[i], 0);
    spriteGates.setTextColor(valColor, COLOR_BG);
    spriteGates.drawString(valBuf, GATE_SLOT_X[i] + 28, 0);
  }

  spriteGates.pushSprite(58, NAVBRK_GATE_Y); // aligns after the static "Gates:" label
}

static void updateSpeedThreshSprite(const NavTelemetry &tel)
{
  if (!g_spriteOk[SPR_SPEED_THRESH]) return;

  // Display-only horizontal speed, computed from the existing filtered N/E velocity -
  // NAVBRAKE's own filtering/control logic is untouched, this is purely a UI convenience
  // (explicitly permitted: "may be calculated for DISPLAY ONLY from filtered N/E").
  float speed = sqrtf((tel.velFiltN * tel.velFiltN) + (tel.velFiltE * tel.velFiltE));

  spriteSpeedThresh.fillSprite(COLOR_BG);
  spriteSpeedThresh.setTextSize(1);
  spriteSpeedThresh.setTextColor(staleColor(tel.navposMs, COLOR_TEXT), COLOR_BG);
  spriteSpeedThresh.setCursor(0, 0);
  char buf[24];
  snprintf(buf, sizeof(buf), "%.2f m/s", speed);
  spriteSpeedThresh.print(buf);

  // Threshold is a static reference value (see NAVBRAKE_MAX_VEL_MPS_DISPLAY_ONLY), so it
  // doesn't need to be part of the change-gated content - stacked below Speed, aligned
  // after the static "Threshold:" label one row down.
  spriteSpeedThresh.setTextColor(COLOR_LABEL, COLOR_BG);
  spriteSpeedThresh.setCursor(0, 18);
  snprintf(buf, sizeof(buf), "%.2f m/s", (double)NAVBRAKE_MAX_VEL_MPS_DISPLAY_ONLY);
  spriteSpeedThresh.print(buf);

  spriteSpeedThresh.pushSprite(70, NAVBRK_ROW3_Y); // aligns after the static "Speed:" label
}

static void updateDiagSprite(const NavTelemetry &tel, uint32_t navHzX10, uint32_t totalLines)
{
  if (!g_spriteOk[SPR_DIAG]) return;

  spriteDiag.fillSprite(COLOR_BG);
  spriteDiag.setTextSize(1);
  spriteDiag.setTextColor(COLOR_LABEL, COLOR_BG);

  char buf[64];
  spriteDiag.setCursor(0, 0);
  snprintf(buf, sizeof(buf), "PKT %lu  AGE %lums  ERR %lu",
           (unsigned long)totalLines, tel.navAgeMs, tel.navRej);
  spriteDiag.print(buf);

  spriteDiag.setCursor(0, 20);
  snprintf(buf, sizeof(buf), "RX %lu.%luHz  DROP %lu",
           (unsigned long)(navHzX10 / 10U), (unsigned long)(navHzX10 % 10U), tel.navDrop);
  spriteDiag.print(buf);

  // Aircraft battery, from the flight controller's own VBAT[...] telemetry (real data,
  // already parsed - just never displayed before). No "tester battery" line: this board
  // (Makerfabs ESP32 TFT Touch w/ Camera v1.2) is USB-only powered per its own schematic -
  // no battery connector or voltage-divider circuit exists on it to read.
  bool vbatFresh = (tel.vbatMs != 0) && ((millis() - tel.vbatMs) < STALE_MS);
  spriteDiag.setTextColor(vbatFresh ? COLOR_TEXT : COLOR_STALE, COLOR_BG);
  spriteDiag.setCursor(0, 40);
  snprintf(buf, sizeof(buf), "Aircraft VBAT %.2fV", tel.vbatMv / 1000.0f);
  spriteDiag.print(buf);

  spriteDiag.pushSprite(4, DIAG_ROW1_Y);
}

static void updateDisplay()
{
  char buf[24];
  NavTelemetry tel;
  bool wifiOk, bridgeOk;
  uint32_t navHzX10, uiHzX10, totalLines;

  {
    TelemetryLock lock(g_telMutex);
    tel = g_tel;
    navHzX10 = g_navHzX10;
    uiHzX10 = g_uiHzX10;
    totalLines = g_totalLines;
  }
  wifiOk = g_wifiConnected;
  bridgeOk = g_bridgeConnected;
  (void)buf;

  lcd.startWrite();
  updateHzSprite(uiHzX10);
  updateGpsStatusSprite(tel, wifiOk, bridgeOk);
  updatePosVelSprites(tel);
  updateModeTitleSprite(tel);
  updateReqActiveSprite(tel);
  updateGateSprite(tel);
  updateSpeedThreshSprite(tel);
  updateDiagSprite(tel, navHzX10, totalLines);
  lcd.endWrite();
}

void setup()
{
  Serial.begin(115200);

  lcd.init();
  lcd.setRotation(0); // portrait - panel is natively 320x480, see the Layout comment above
  initDisplaySprites();
  drawStaticLayout();

  g_telMutex = xSemaphoreCreateMutex();
  g_cmdMutex = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  xTaskCreatePinnedToCore(bridgeTask, "bridgeRx", 8192, nullptr, 2, nullptr, 0);
}

void loop()
{
  static uint32_t lastRateMs = 0;
  static uint32_t lastUi = 0;
  static uint32_t lastTelArmStatusMs = 0;

  uint32_t now = millis();

  // Periodic resync: RC channel 7 on the STM32 side is the actual control, this just
  // keeps the on-screen Tel:ON/off indicator trustworthy without ever writing anything.
  if ((now - lastTelArmStatusMs) >= 3000U)
  {
    lastTelArmStatusMs = now;
    queueCommand("TELARM STATUS");
  }

  if (lastRateMs == 0)
  {
    lastRateMs = now;
  }
  else if ((now - lastRateMs) >= 1000U)
  {
    uint32_t elapsedMs = now - lastRateMs;
    uint32_t freshNavEvents;
    {
      TelemetryLock lock(g_telMutex);
      freshNavEvents = g_freshNavEvents;
      g_freshNavEvents = 0;
    }
    g_navHzX10 = (freshNavEvents * 10000U) / elapsedMs;
    g_uiHzX10 = (g_uiFrames * 10000U) / elapsedMs;
    g_uiFrames = 0;
    lastRateMs = now;
  }

  // Non-blocking display pacing (no delay()) - decoupled from GPS/UART/bridge timing,
  // which all run on bridgeTask (core 0) regardless of display refresh rate.
  if ((now - lastUi) >= UI_REFRESH_MS)
  {
    lastUi = now;
    g_uiFrames++;
    updateDisplay();
  }
}
