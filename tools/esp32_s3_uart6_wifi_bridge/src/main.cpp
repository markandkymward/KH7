#include <WiFi.h>
#include <ESPmDNS.h>
#include <errno.h>
#include <lwip/sockets.h>
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
// How many TCP clients can be bridged to the FC's UART at once (e.g. the desktop
// GUI and the ESP32 field-tester display connected at the same time). UART data is
// broadcast to all connected clients; inbound bytes from any client are merged onto
// the single UART TX (fine in practice - only one side normally sends commands at a time).
#define BRIDGE_MAX_CLIENTS 3
#define BRIDGE_CLIENT_TX_BUFFER_BYTES 16384U
#define BRIDGE_CLIENT_FLUSH_BUDGET_BYTES 4096U

WiFiServer server(BRIDGE_PORT);
WiFiClient clients[BRIDGE_MAX_CLIENTS];
static bool client_was_connected[BRIDGE_MAX_CLIENTS] = {false};
static uint8_t client_tx_buffers[BRIDGE_MAX_CLIENTS][BRIDGE_CLIENT_TX_BUFFER_BYTES];
static size_t client_tx_head[BRIDGE_MAX_CLIENTS] = {0};
static size_t client_tx_tail[BRIDGE_MAX_CLIENTS] = {0};
static size_t client_tx_count[BRIDGE_MAX_CLIENTS] = {0};

static uint32_t last_status_ms = 0;
static uint32_t last_uart_rx_ms = 0;
static uint32_t last_wifi_retry_ms = 0;
static uint32_t wifi_connect_start_ms = 0;
static uint32_t last_heartbeat_ms = 0;
static uint32_t activity_until_ms = 0;
static uint32_t uart_to_tcp_bytes = 0;
static uint32_t tcp_to_uart_bytes = 0;
static uint32_t client_dropped_bytes[BRIDGE_MAX_CLIENTS] = {0};
static bool mdns_started = false;
static bool wifi_connect_in_progress = false;
static bool led_state = false;

static const char *wifi_status_text(wl_status_t status);

static int bridge_connected_client_count()
{
  int count = 0;
  for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
  {
    if (clients[i].connected())
    {
      count++;
    }
  }
  return count;
}

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

static void bridge_client_reset_tx(int client_index)
{
  client_tx_head[client_index] = 0;
  client_tx_tail[client_index] = 0;
  client_tx_count[client_index] = 0;
}

static bool bridge_client_queue(int client_index, const uint8_t *data, size_t length)
{
  if (length > (BRIDGE_CLIENT_TX_BUFFER_BYTES - client_tx_count[client_index]))
  {
    client_dropped_bytes[client_index] += (uint32_t)length;
    clients[client_index].stop();
    bridge_client_reset_tx(client_index);
    return false;
  }

  for (size_t i = 0; i < length; i++)
  {
    client_tx_buffers[client_index][client_tx_head[client_index]] = data[i];
    client_tx_head[client_index] = (client_tx_head[client_index] + 1U) % BRIDGE_CLIENT_TX_BUFFER_BYTES;
  }
  client_tx_count[client_index] += length;
  return true;
}

static size_t bridge_client_flush(int client_index)
{
  size_t total_sent = 0;
  int socket_fd = clients[client_index].fd();

  while ((socket_fd >= 0) && (client_tx_count[client_index] > 0U) &&
         (total_sent < BRIDGE_CLIENT_FLUSH_BUDGET_BYTES))
  {
    size_t contiguous = BRIDGE_CLIENT_TX_BUFFER_BYTES - client_tx_tail[client_index];
    size_t budget_remaining = BRIDGE_CLIENT_FLUSH_BUDGET_BYTES - total_sent;
    int sent;

    if (contiguous > client_tx_count[client_index])
    {
      contiguous = client_tx_count[client_index];
    }
    if (contiguous > budget_remaining)
    {
      contiguous = budget_remaining;
    }

    sent = send(socket_fd, &client_tx_buffers[client_index][client_tx_tail[client_index]],
                contiguous, MSG_DONTWAIT);
    if (sent > 0)
    {
      size_t sent_size = (size_t)sent;
      client_tx_tail[client_index] = (client_tx_tail[client_index] + sent_size) % BRIDGE_CLIENT_TX_BUFFER_BYTES;
      client_tx_count[client_index] -= sent_size;
      total_sent += sent_size;
    }
    else if ((sent < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK))
    {
      clients[client_index].stop();
      bridge_client_reset_tx(client_index);
      break;
    }
    else
    {
      break;
    }
  }

  return total_sent;
}

static void bridge_client_printf(const char *fmt, ...)
{
  char buf[192];
  va_list args;

  va_start(args, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
  {
    if (clients[i].connected())
    {
      size_t length = strlen(buf);
      (void)bridge_client_queue(i, (const uint8_t *)buf, length);
    }
  }
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
    for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
    {
      if (!clients[i].connected())
      {
        client_was_connected[i] = false;
      }
    }
    update_activity_led();
    delay(1);
    return;
  }

  // Detect disconnects and free the slot.
  for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
  {
    if (client_was_connected[i] && !clients[i].connected())
    {
      client_was_connected[i] = false;
      bridge_log_printf("[TCP] Client %d disconnected\r\n", i);
      clients[i].stop();
      bridge_client_reset_tx(i);
    }
  }

  // Accept a new connection into any free slot; reject if all slots are busy.
  if (server.hasClient())
  {
    WiFiClient next = server.available();
    int slot = -1;
    for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
    {
      if (!clients[i].connected())
      {
        slot = i;
        break;
      }
    }

    if (slot < 0)
    {
      bridge_log_printf("[TCP] Rejecting new client, all %d slots full\r\n", BRIDGE_MAX_CLIENTS);
      next.stop();
    }
    else
    {
      clients[slot] = next;
      clients[slot].setNoDelay(true);
      client_dropped_bytes[slot] = 0;
      bridge_client_reset_tx(slot);
      client_was_connected[slot] = true;
      bridge_log_printf("[TCP] Client %d connected: %s:%u (now %d/%d)\r\n",
                    slot,
                    clients[slot].remoteIP().toString().c_str(),
                    clients[slot].remotePort(),
                    bridge_connected_client_count(),
                    BRIDGE_MAX_CLIENTS);
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

  for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
  {
    if (clients[i].connected())
    {
      (void)bridge_client_flush(i);
    }
  }

  // UART -> broadcast to every connected client.
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

    bool any_queued = false;
    for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
    {
      if (clients[i].connected())
      {
        if (bridge_client_queue(i, buffer, (size_t)count))
        {
          any_queued = true;
        }
      }
    }
    uart_to_tcp_bytes += (uint32_t)count;
    last_uart_rx_ms = millis();
    if (any_queued)
    {
      pulse_activity_led();
    }
  }

  for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
  {
    if (clients[i].connected())
    {
      (void)bridge_client_flush(i);
    }
  }

  // Any client -> UART, merged onto the single TX line (see BRIDGE_MAX_CLIENTS comment).
  for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
  {
    while (clients[i].connected() && (clients[i].available() > 0) && (Serial2.availableForWrite() > 0))
    {
      uint8_t b = (uint8_t)clients[i].read();
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
    bridge_log_printf("[STAT] wifi=%s ip=%s rssi=%d clients=%d/%d u2t=%lu t2u=%lu q=[%u %u %u] drop=[%lu %lu %lu]\r\n",
                  wifi_status_text(WiFi.status()),
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(),
                  bridge_connected_client_count(),
                  BRIDGE_MAX_CLIENTS,
                  (unsigned long)uart_to_tcp_bytes,
            (unsigned long)tcp_to_uart_bytes,
            (unsigned int)client_tx_count[0],
            (unsigned int)client_tx_count[1],
            (unsigned int)client_tx_count[2],
            (unsigned long)client_dropped_bytes[0],
            (unsigned long)client_dropped_bytes[1],
            (unsigned long)client_dropped_bytes[2]);

    /* Found 2026-08-16: this client-facing broadcast shares the same per-client
     * TX ring buffer as UART-relayed bytes (both go through bridge_client_queue()),
     * and used to fire on a strict 1-second wall clock with no idea whether a
     * relayed line (e.g. a SDLOG DUMP block's ~1KB hex line) was still mid-transfer -
     * confirmed in a real capture: hex output cut off mid-line, "[BRIDGE] STAT..."
     * spliced in, then the rest of the block's hex resumed after it on later loop()
     * iterations.
     *
     * First attempt (also 2026-08-16) gated this on Serial2.available() == 0 - which
     * turned out to be nearly always true right here regardless of whether a transfer
     * was still going, since it's checked right after the drain loop above already
     * emptied it THIS iteration; it only ever tells you "this ~1-2ms tick is done",
     * never "the whole multi-second transfer is done". Confirmed by the truncations
     * still landing ~every 11 blocks - essentially still every ~1s tick, i.e. the
     * check wasn't suppressing anything. What actually distinguishes "still streaming"
     * from "genuinely idle" is recency: at 115200 baud a live multi-block dump refills
     * Serial2's RX every couple ms (the STM32 side re-fills its own TX queue just as
     * it drains), so last_uart_rx_ms stays fresh throughout: normal (non-dump)
     * telemetry has real ~100ms+ gaps between bursts (APP_IMU_TELEMETRY_MS), so a
     * 50ms idle requirement finds plenty of safe windows during normal operation but
     * blocks for a live dump's entire duration. */
    if ((millis() - last_uart_rx_ms) >= 50U)
    {
      bool relay_idle = true;

      for (int i = 0; i < BRIDGE_MAX_CLIENTS; i++)
      {
        if (clients[i].connected() && (client_tx_count[i] != 0U))
        {
          relay_idle = false;
          break;
        }
      }

      if (relay_idle)
      {
        bridge_client_printf("[BRIDGE] STAT wifi=%s rssi=%d u2t=%lu t2u=%lu\r\n",
                             wifi_status_text(WiFi.status()),
                             WiFi.RSSI(),
                             (unsigned long)uart_to_tcp_bytes,
                             (unsigned long)tcp_to_uart_bytes);
      }
    }
  }

  update_activity_led();
  delay(1);
}
