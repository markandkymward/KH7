#include <WiFi.h>
#include <ESPmDNS.h>
#include <stdarg.h>
#include <stdio.h>

static void bridge_log_line(const char *msg)
{
  Serial.println(msg);
}

static void bridge_log_printf(const char *fmt, ...)
{
  char buf[192];
  va_list args;

  va_start(args, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  Serial.print(buf);
}

static const char *WIFI_SSID = "NachoWi-Fi";
static const char *WIFI_PASSWORD = "NrMaintain1!";

static const uint32_t FC_UART_BAUD = 115200;
static const int FC_UART_RX_PIN = 16;
static const int FC_UART_TX_PIN = 17;
static const uint16_t BRIDGE_PORT = 3333;

WiFiServer server(BRIDGE_PORT);
WiFiClient client;

static uint32_t last_status_ms = 0;
static bool mdns_started = false;
static bool had_client = false;

static const char *wifi_status_text(wl_status_t status)
{
  switch (status)
  {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID";
    case WL_SCAN_COMPLETED:
      return "SCAN_DONE";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

static void connect_wifi()
{
  uint32_t start_ms;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  bridge_log_printf("[WIFI] Connecting to SSID '%s'...\r\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  start_ms = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    if ((millis() - start_ms) > 10000U)
    {
      bridge_log_printf("[WIFI] Still connecting... status=%s\r\n", wifi_status_text(WiFi.status()));
      start_ms = millis();
    }
    delay(500);
  }

  bridge_log_printf("[WIFI] Connected. IP=%s RSSI=%d dBm\r\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

  if (!mdns_started)
  {
    if (MDNS.begin("kh7bridge"))
    {
      MDNS.addService("kh7", "tcp", BRIDGE_PORT);
      mdns_started = true;
      bridge_log_printf("[MDNS] Hostname available: kh7bridge.local\r\n");
    }
    else
    {
      bridge_log_printf("[MDNS] Failed to start mDNS\r\n");
    }
  }
}

void setup()
{
  uint32_t serial_wait_start;

  Serial.begin(115200);
  serial_wait_start = millis();
  while (!Serial && ((millis() - serial_wait_start) < 3000U))
  {
    delay(10);
  }
  delay(200);
  bridge_log_line("[BOOT] ESP32-S3 UART6 Wi-Fi bridge starting");
  Serial2.begin(FC_UART_BAUD, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);
  bridge_log_printf("[UART2] FC bridge baud=%lu RX=%d TX=%d\r\n", FC_UART_BAUD, FC_UART_RX_PIN, FC_UART_TX_PIN);

  connect_wifi();
  server.begin();
  server.setNoDelay(true);
  bridge_log_printf("[TCP] Listening on port %u\r\n", BRIDGE_PORT);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    bridge_log_printf("[WIFI] Disconnected (status=%s), reconnecting...\r\n", wifi_status_text(WiFi.status()));
    connect_wifi();
  }

  if (!client || !client.connected())
  {
    if (had_client)
    {
      had_client = false;
      bridge_log_line("[TCP] Client disconnected");
    }

    WiFiClient next = server.available();
    if (next)
    {
      client.stop();
      client = next;
      client.setNoDelay(true);
      had_client = true;
      bridge_log_printf("[TCP] Client connected: %s:%u\r\n",
                    client.remoteIP().toString().c_str(),
                    client.remotePort());
    }
  }

  if (client && client.connected())
  {
    while (Serial2.available() > 0)
    {
      uint8_t b = (uint8_t)Serial2.read();
      client.write(&b, 1);
    }

    while (client.available() > 0)
    {
      uint8_t b = (uint8_t)client.read();
      Serial2.write(&b, 1);
    }
  }

  if ((millis() - last_status_ms) > 1000)
  {
    last_status_ms = millis();
    bridge_log_printf("[STAT] wifi=%s ip=%s rssi=%d client=%s\r\n",
                  wifi_status_text(WiFi.status()),
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(),
                  (client && client.connected()) ? "yes" : "no");
  }
}
