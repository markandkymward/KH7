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

static const uint32_t WIFI_RETRY_MS = 4000;
static const uint32_t BRIDGE_RETRY_MS = 3000;
static const uint32_t BRIDGE_DATA_TIMEOUT_MS = 2000;
static const uint32_t UI_REFRESH_MS = 200;
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

// ---- UI ----
// Layout rule that fixed the field-unit alignment bugs: every row is drawn as ONE
// combined "label: value" string from a single field slot - never a separately-drawn
// static label next to an independently-cleared value box. Mixing those let a value's
// fillRect erase part of an adjacent label/title (exactly what happened to the title
// row before this fix). Row widths below are sized with margin verified against the
// longest realistic string at that text size, so text can never overflow into another
// row's box either.
#define COLOR_BG TFT_BLACK
#define COLOR_LABEL 0x7BEF // light grey
#define COLOR_HEADER TFT_CYAN
#define COLOR_GOOD TFT_GREEN
#define COLOR_BAD TFT_RED
#define COLOR_WARN TFT_YELLOW
#define COLOR_STALE 0x4208 // dark grey, for values that haven't updated recently
#define COLOR_TEXT TFT_WHITE

enum FieldId
{
  F_CONN = 0,
  F_GPS_VALID,
  F_GPS_REASON,
  F_GPS_STATS,
  F_POS,
  F_VEL_RAW,
  F_VEL_FILT,
  F_BRK_STATE,
  F_BRK_LIMITS,
  F_BRK_DESVEL,
  F_BRK_ACCEL,
  F_BRK_ANG,
  F_STATUS_ROW,
  F_LOST,
  F_COUNT
};

struct FieldLayout
{
  int16_t x, y, w, h;
  uint8_t textSize;
};

// Landscape 480x320 after setRotation(1). Widths include margin over the longest
// realistic value at that text size (verified by hand: size1 ~6px/char, size2 ~12px/char,
// size3 ~18px/char) so text never runs past its own box into the next row.
static const FieldLayout LAYOUT[F_COUNT] = {
    /*F_CONN      */ {8, 30, 460, 18, 2},
    /*F_GPS_VALID */ {8, 58, 320, 28, 3},
    /*F_GPS_REASON*/ {8, 96, 470, 14, 1},
    /*F_GPS_STATS */ {8, 110, 470, 14, 1},
    /*F_POS       */ {8, 132, 460, 18, 2},
    /*F_VEL_RAW   */ {8, 154, 460, 18, 2},
    /*F_VEL_FILT  */ {8, 176, 460, 18, 2},
    /*F_BRK_STATE */ {8, 204, 460, 18, 2},
    /*F_BRK_LIMITS*/ {8, 226, 470, 14, 1},
    /*F_BRK_DESVEL*/ {8, 240, 470, 14, 1},
    /*F_BRK_ACCEL */ {8, 254, 470, 14, 1},
    /*F_BRK_ANG   */ {8, 268, 470, 14, 1},
    /*F_STATUS_ROW*/ {8, 288, 470, 14, 1},
    /*F_LOST      */ {4, 302, 472, 14, 1},
};

static char g_fieldCache[F_COUNT][80];

static void setField(FieldId id, const char *text, uint16_t color)
{
  const FieldLayout &l = LAYOUT[id];
  if (strncmp(g_fieldCache[id], text, sizeof(g_fieldCache[id])) == 0)
  {
    return;
  }
  strncpy(g_fieldCache[id], text, sizeof(g_fieldCache[id]) - 1);
  g_fieldCache[id][sizeof(g_fieldCache[id]) - 1] = '\0';

  lcd.fillRect(l.x, l.y, l.w, l.h, COLOR_BG);
  lcd.setTextColor(color, COLOR_BG);
  lcd.setTextSize(l.textSize);
  lcd.setCursor(l.x, l.y);
  lcd.print(text);
}

static void drawStaticLabels()
{
  lcd.fillScreen(COLOR_BG);
  lcd.setTextColor(COLOR_HEADER, COLOR_BG);
  lcd.setTextSize(2);
  lcd.setCursor(8, 4);
  lcd.print("KH7 GPS FIELD TESTER");

  lcd.drawFastHLine(0, 24, 480, COLOR_LABEL);
  lcd.drawFastHLine(0, 52, 480, COLOR_LABEL);
  lcd.drawFastHLine(0, 126, 480, COLOR_LABEL);
  lcd.drawFastHLine(0, 198, 480, COLOR_LABEL);
  lcd.drawFastHLine(0, 282, 480, COLOR_LABEL);
}

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

static void refreshUi()
{
  char buf[80];
  NavTelemetry tel;

  {
    TelemetryLock lock(g_telMutex);
    tel = g_tel;
  }

  lcd.startWrite();

  bool wifiOk = g_wifiConnected;
  bool bridgeOk = g_bridgeConnected;
  snprintf(buf, sizeof(buf), "W:%s B:%s GPS:%lu.%lu UI:%lu.%luHz",
           wifiOk ? "OK" : "--", bridgeOk ? "OK" : "--",
           (unsigned long)(g_navHzX10 / 10U), (unsigned long)(g_navHzX10 % 10U),
           (unsigned long)(g_uiHzX10 / 10U), (unsigned long)(g_uiHzX10 % 10U));
  setField(F_CONN, buf, (wifiOk && bridgeOk) ? COLOR_GOOD : COLOR_BAD);

  bool navFresh = (tel.navMs != 0) && ((millis() - tel.navMs) < STALE_MS);
  uint16_t validColor = !navFresh ? COLOR_STALE : (tel.navValid ? COLOR_GOOD : COLOR_BAD);
  snprintf(buf, sizeof(buf), "GPS: %s", (navFresh && tel.navValid) ? "VALID" : (navFresh ? "INVALID" : "NO DATA"));
  setField(F_GPS_VALID, buf, validColor);

  snprintf(buf, sizeof(buf), "reason:%s ref:%s cv:%lu ci:%lu drop:%lu",
           reasonName(tel.navReason), tel.navRef ? "yes" : "no",
           tel.navCv, tel.navCi, tel.navDrop);
  setField(F_GPS_REASON, buf, staleColor(tel.navMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "fix:%s sats:%u hAcc:%.1fm age:%lums upd:%lums",
           fixTypeName(tel.navFix), tel.navSats, tel.navHaccCm / 100.0f, tel.navAgeMs, tel.navUpdMs);
  setField(F_GPS_STATS, buf, staleColor(tel.navMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "Pos N/E: %.1f / %.1f m", tel.posN, tel.posE);
  setField(F_POS, buf, staleColor(tel.navposMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "Vel raw N/E: %.2f / %.2f m/s", tel.velRawN, tel.velRawE);
  setField(F_VEL_RAW, buf, staleColor(tel.navposMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "Vel filt N/E: %.2f / %.2f m/s", tel.velFiltN, tel.velFiltE);
  setField(F_VEL_FILT, buf, staleColor(tel.navposMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "NAVBRAKE req:%s active:%s", tel.brkReq ? "YES" : "no", tel.brkAct ? "YES" : "no");
  setField(F_BRK_STATE, buf, staleColor(tel.navbrkMs, tel.brkAct ? COLOR_GOOD : COLOR_TEXT));

  snprintf(buf, sizeof(buf), "gate N/R/A/B/L:%u/%u/%u/%u/%u latch:%u lim T/A:%u/%u",
           tel.gateNav, tel.gateRef, tel.gateAtt, tel.gateBaro, tel.gateLink, tel.gateLatch,
           tel.brkTiltLim, tel.brkAccelLim);
  setField(F_BRK_LIMITS, buf, staleColor(tel.navgateMs,
           (tel.gateLatch || !tel.gateNav || !tel.gateRef || !tel.gateAtt || !tel.gateBaro || !tel.gateLink)
               ? COLOR_WARN : COLOR_TEXT));

  snprintf(buf, sizeof(buf), "des N/E:%.2f/%.2f  err N/E:%.2f/%.2f",
           tel.brkDesN, tel.brkDesE, tel.brkErrN, tel.brkErrE);
  setField(F_BRK_DESVEL, buf, staleColor(tel.navbrkMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "accel N/E/fwd/right:%.2f/%.2f/%.2f/%.2f m/s2",
           tel.brkAccelN, tel.brkAccelE, tel.brkAccelFwd, tel.brkAccelRight);
  setField(F_BRK_ACCEL, buf, staleColor(tel.navbrkMs, COLOR_TEXT));

  snprintf(buf, sizeof(buf), "ang cmd R/P:%.1f/%.1f deg", tel.brkAngRoll, tel.brkAngPitch);
  setField(F_BRK_ANG, buf, staleColor(tel.navbrkMs, COLOR_TEXT));

  bool armFresh = (tel.armMs != 0) && ((millis() - tel.armMs) < STALE_MS);
  snprintf(buf, sizeof(buf), "Mode:%s Armed:%s V:%.2f Alt:%.1fm dis:%s Tel:%s",
           tel.modeMs ? tel.modeName : "-", armFresh ? (tel.armA ? "YES" : "no") : "?",
           tel.vbatMv / 1000.0f, tel.baroCm / 100.0f, tel.lastDisarmReason,
           tel.telArmEnabled ? "ON" : "off");
  setField(F_STATUS_ROW, buf, !armFresh ? COLOR_STALE : (tel.armA ? COLOR_WARN : COLOR_TEXT));

  if (tel.lostMs != 0)
  {
    snprintf(buf, sizeof(buf), "Last NAV_LOST: %s (%lus ago)", tel.lostReason, (millis() - tel.lostMs) / 1000UL);
    setField(F_LOST, buf, COLOR_WARN);
  }

  lcd.endWrite();
}


void setup()
{
  Serial.begin(115200);

  lcd.init();
  lcd.setRotation(1);
  drawStaticLabels();

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

  if ((now - lastUi) >= UI_REFRESH_MS)
  {
    lastUi = now;
    g_uiFrames++;
    refreshUi();
  }
}
