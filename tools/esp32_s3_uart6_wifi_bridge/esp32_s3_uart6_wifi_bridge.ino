#include <WiFi.h>

static const char *WIFI_SSID = "NachWi-Fi";
static const char *WIFI_PASSWORD = "NrMaintain1!";

static const uint32_t FC_UART_BAUD = 115200;
static const int FC_UART_RX_PIN = 44;
static const int FC_UART_TX_PIN = 43;
static const uint16_t BRIDGE_PORT = 3333;

WiFiServer server(BRIDGE_PORT);
WiFiClient client;

static uint32_t last_status_ms = 0;

static void connect_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
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

  if ((millis() - last_status_ms) > 3000)
  {
    last_status_ms = millis();
  }
}
