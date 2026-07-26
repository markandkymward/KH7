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
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000U;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 2500U;

static const uint32_t FC_UART_BAUD = 115200;
static const int FC_UART_RX_PIN = 16;
static const int FC_UART_TX_PIN = 17;
static const uint16_t BRIDGE_PORT = 3333;
#if defined(LED_BUILTIN)
static const int ACTIVITY_LED_PIN = LED_BUILTIN;
#else
static const int ACTIVITY_LED_PIN = 2;
#endif
static const uint32_t ACTIVITY_HOLD_MS = 80U;
static const uint32_t HEARTBEAT_MS = 500U;

WiFiServer server(BRIDGE_PORT);
WiFiClient client;

static uint32_t last_status_ms = 0;
static uint32_t last_wifi_retry_ms = 0;
static uint32_t wifi_connect_start_ms = 0;
static uint32_t last_heartbeat_ms = 0;
static uint32_t activity_until_ms = 0;
static uint32_t uart_to_tcp_bytes = 0;
static uint32_t tcp_to_uart_bytes = 0;
static bool mdns_started = false;
static bool had_client = false;
static bool wifi_connect_in_progress = false;
static bool led_state = false;

static const char *wifi_status_text(wl_status_t status);

static bool try_connect_wifi_ssid(const char *ssid)
{
  uint32_t start_ms = millis();
  wl_status_t status;

  bridge_log_printf("[WIFI] Connecting to SSID '%s'...\r\n", ssid);
  WiFi.begin(ssid, WIFI_PASSWORD);

  while ((millis() - start_ms) < WIFI_CONNECT_TIMEOUT_MS)
  {
    status = WiFi.status();
    if (status == WL_CONNECTED)
    {
      bridge_log_printf("[WIFI] Connected to '%s'. IP=%s RSSI=%d dBm\r\n",
                        ssid,
                        WiFi.localIP().toString().c_str(),
                        WiFi.RSSI());
      return true;
    }

    if (status == WL_CONNECT_FAILED)
    {
      bridge_log_printf("[WIFI] Connect failed for '%s'\r\n", ssid);
      break;
    }

    delay(250);
  }

  bridge_log_printf("[WIFI] Timeout on '%s' status=%s\r\n",
                    ssid,
                    wifi_status_text(WiFi.status()));
  return false;
}

static void bridge_client_printf(const char *fmt, ...)
{
  char buf[192];
  va_list args;

  if (!client || !client.connected())
  {
    return;
  }

  va_start(args, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  client.print(buf);
}

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

static bool connect_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  WiFi.disconnect(true, true);
  delay(150);

  if (try_connect_wifi_ssid(WIFI_SSID))
  {
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

    return true;
  }

  bridge_log_printf("[WIFI] Unable to connect to configured SSID '%s'\r\n", WIFI_SSID);
  return false;
}

static void start_wifi_connect_attempt()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  bridge_log_printf("[WIFI] Connecting to SSID '%s'...\r\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifi_connect_start_ms = millis();
  last_wifi_retry_ms = wifi_connect_start_ms;
  wifi_connect_in_progress = true;
}

static void pulse_activity_led()
{
  activity_until_ms = millis() + ACTIVITY_HOLD_MS;
}

static void update_activity_led()
{
  bool desired = led_state;

  if (millis() < activity_until_ms)
  {
    desired = true;
  }

  digitalWrite(ACTIVITY_LED_PIN, desired ? HIGH : LOW);
}

static void service_wifi_state()
{
  wl_status_t status = WiFi.status();
  uint32_t now_ms = millis();

  if (status == WL_CONNECTED)
  {
    if (wifi_connect_in_progress)
    {
      wifi_connect_in_progress = false;
      bridge_log_printf("[WIFI] Connected to '%s'. IP=%s RSSI=%d dBm\r\n",
                        WIFI_SSID,
                        WiFi.localIP().toString().c_str(),
                        WiFi.RSSI());
    }

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

    return;
  }

  if (wifi_connect_in_progress)
  {
    if ((now_ms - wifi_connect_start_ms) >= WIFI_CONNECT_TIMEOUT_MS)
    {
      wifi_connect_in_progress = false;
      bridge_log_printf("[WIFI] Timeout on '%s' status=%s\r\n",
                        WIFI_SSID,
                        wifi_status_text(status));
    }
    return;
  }

  if ((now_ms - last_wifi_retry_ms) >= WIFI_RETRY_INTERVAL_MS)
  {
    bridge_log_printf("[WIFI] Disconnected (status=%s), reconnecting...\r\n",
                      wifi_status_text(status));
    start_wifi_connect_attempt();
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
  pinMode(ACTIVITY_LED_PIN, OUTPUT);
  digitalWrite(ACTIVITY_LED_PIN, LOW);
  Serial2.begin(FC_UART_BAUD, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);
  bridge_log_printf("[UART2] FC bridge baud=%lu RX=%d TX=%d\r\n", FC_UART_BAUD, FC_UART_RX_PIN, FC_UART_TX_PIN);

  (void)connect_wifi();
  wifi_connect_in_progress = false;
  last_wifi_retry_ms = millis();
  server.begin();
  server.setNoDelay(true);
  bridge_log_printf("[TCP] Listening on port %u\r\n", BRIDGE_PORT);
}

void loop()
{
  service_wifi_state();

  if ((millis() - last_heartbeat_ms) >= HEARTBEAT_MS)
  {
    last_heartbeat_ms = millis();
    led_state = !led_state;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    if (!client || !client.connected())
    {
      had_client = false;
    }
    update_activity_led();
    delay(1);
    return;
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
      bridge_client_printf("[WIFI] CONFIG ssid=%s\r\n", WIFI_SSID);
      bridge_client_printf("[WIFI] STATE status=%s ip=%s rssi=%d\r\n",
               wifi_status_text(WiFi.status()),
               WiFi.localIP().toString().c_str(),
               WiFi.RSSI());
      bridge_client_printf("[BRIDGE] CONNECTED wifi=%s ip=%s uart_rx=%d uart_tx=%d\r\n",
                           wifi_status_text(WiFi.status()),
                           WiFi.localIP().toString().c_str(),
                           FC_UART_RX_PIN,
                           FC_UART_TX_PIN);
    }
  }

  if (client && client.connected())
  {
    while (Serial2.available() > 0)
    {
      uint8_t buffer[128];
      int available = Serial2.available();
      int to_read = (available > (int)sizeof(buffer)) ? (int)sizeof(buffer) : available;
      int count = Serial2.readBytes(buffer, to_read);
      if (count <= 0)
      {
        break;
      }

      size_t sent = client.write(buffer, (size_t)count);
      uart_to_tcp_bytes += (uint32_t)sent;
      if (sent > 0)
      {
        pulse_activity_led();
      }
      if (sent < (size_t)count)
      {
        break;
      }
    }

    while ((client.available() > 0) && (Serial2.availableForWrite() > 0))
    {
      uint8_t b = (uint8_t)client.read();
      size_t written = Serial2.write(&b, 1);
      if (written == 0)
      {
        break;
      }
      tcp_to_uart_bytes += (uint32_t)written;
      pulse_activity_led();
    }
  }

  if ((millis() - last_status_ms) > 1000)
  {
    last_status_ms = millis();
    bridge_log_printf("[STAT] wifi=%s ip=%s rssi=%d client=%s u2t=%lu t2u=%lu\r\n",
                  wifi_status_text(WiFi.status()),
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(),
                  (client && client.connected()) ? "yes" : "no",
                  (unsigned long)uart_to_tcp_bytes,
                  (unsigned long)tcp_to_uart_bytes);
    bridge_client_printf("[BRIDGE] STAT wifi=%s rssi=%d u2t=%lu t2u=%lu\r\n",
                         wifi_status_text(WiFi.status()),
                         WiFi.RSSI(),
                         (unsigned long)uart_to_tcp_bytes,
                         (unsigned long)tcp_to_uart_bytes);
  }

  update_activity_led();
  delay(1);
}
