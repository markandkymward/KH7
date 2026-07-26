#include <WiFi.h>

static const char *WIFI_SSID = "NachoWi-Fi";
static const char *WIFI_PASSWORD = "NrMaintain1!";

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

void setup()
{
  Serial.begin(115200);
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
    while (Serial2.available() > 0)
    {
      uint8_t b = (uint8_t)Serial2.read();
      client.write(&b, 1);
      uart_to_tcp_bytes++;
      Serial.printf("[ESP32] UART→TCP: 0x%02X ('%c')\r\n", b, (b >= 32 && b < 127) ? (char)b : '?');
    }

    while (client.available() > 0)
    {
      uint8_t b = (uint8_t)client.read();
      Serial2.write(&b, 1);
      tcp_to_uart_bytes++;
      Serial.printf("[ESP32] TCP→UART: 0x%02X ('%c')\r\n", b, (b >= 32 && b < 127) ? (char)b : '?');
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
