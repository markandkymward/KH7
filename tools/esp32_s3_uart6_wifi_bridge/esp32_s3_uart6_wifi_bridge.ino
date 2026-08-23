#include <WiFi.h>

static const char *WIFI_SSID = "NachoWi-Fi";
static const char *WIFI_PASSWORD = "NrMaintain1!";

/* Must match huart6.Init.BaudRate in Core/Src/main.c on the flight controller.
 * REVERTED to 115200 on 2026-08-22 after 921600 corrupted the UART6 payload
 * (likely a clock-divisor accuracy issue on the STM32 side) - see the comment
 * on huart6.Init.BaudRate in main.c for the full writeup. */
static const uint32_t FC_UART_BAUD = 115200;
static const int FC_UART_RX_PIN = 16;
static const int FC_UART_TX_PIN = 17;
static const uint16_t BRIDGE_PORT = 3333;

WiFiServer server(BRIDGE_PORT);
WiFiClient client;

static uint32_t last_status_ms = 0;
static uint32_t uart_to_tcp_bytes = 0;
static uint32_t tcp_to_uart_bytes = 0;

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

static void connect_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.println("[ESP32] Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n[ESP32] WiFi connected!");
  Serial.print("[ESP32] IP: ");
  Serial.println(WiFi.localIP());
}

/* Byte-by-byte relaying (one client.write()/Serial2.write() call, plus one
 * Serial.printf() debug line, PER BYTE) was the actual bottleneck for SD log
 * pulls - measured ~8.8KB/s, right at the UART's raw baud ceiling, because each
 * printf() call (formatting + a blocking USB-serial write) took far longer than
 * one UART bit-time, so debug logging was throttling the relay to barely UART
 * speed regardless of how fast the far ends could go. Found/fixed 2026-08-22.
 * Draining each side into a local buffer and doing ONE write() per loop pass
 * removes that bottleneck; the buffer size just needs to comfortably exceed
 * what one loop() iteration can accumulate. */
static uint8_t s_uart_to_tcp_buf[512];
static uint8_t s_tcp_to_uart_buf[512];

void setup()
{
  Serial.begin(115200);
  Serial2.setRxBufferSize(2048);
  Serial2.begin(FC_UART_BAUD, SERIAL_8N1, FC_UART_RX_PIN, FC_UART_TX_PIN);

  connect_wifi();
  server.begin();
  server.setNoDelay(true);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connect_wifi();
  }

  if (!client || !client.connected())
  {
    WiFiClient next = server.available();
    if (next)
    {
      client.stop();
      client = next;
      client.setNoDelay(true);
      Serial.println("[ESP32] TCP Client connected!");
      bridge_client_printf("[BRIDGE] CONNECTED ip=%s uart_rx=%d uart_tx=%d\r\n",
                           WiFi.localIP().toString().c_str(),
                           FC_UART_RX_PIN,
                           FC_UART_TX_PIN);
    }
  }

  if (client && client.connected())
  {
    int n = 0;
    while (Serial2.available() > 0 && n < (int)sizeof(s_uart_to_tcp_buf))
    {
      s_uart_to_tcp_buf[n++] = (uint8_t)Serial2.read();
    }
    if (n > 0)
    {
      client.write(s_uart_to_tcp_buf, n);
      uart_to_tcp_bytes += (uint32_t)n;
    }

    int m = 0;
    while (client.available() > 0 && m < (int)sizeof(s_tcp_to_uart_buf))
    {
      s_tcp_to_uart_buf[m++] = (uint8_t)client.read();
    }
    if (m > 0)
    {
      Serial2.write(s_tcp_to_uart_buf, m);
      tcp_to_uart_bytes += (uint32_t)m;
    }
  }

  if ((millis() - last_status_ms) > 3000)
  {
    last_status_ms = millis();
    Serial.printf("[ESP32] STAT wifi=%d client=%s u2t=%lu t2u=%lu\r\n",
                  (int)WiFi.status(),
                  (client && client.connected()) ? "yes" : "no",
                  (unsigned long)uart_to_tcp_bytes,
                  (unsigned long)tcp_to_uart_bytes);
    bridge_client_printf("[BRIDGE] STAT wifi=%d u2t=%lu t2u=%lu\r\n",
                         (int)WiFi.status(),
                         (unsigned long)uart_to_tcp_bytes,
                         (unsigned long)tcp_to_uart_bytes);
  }
}
