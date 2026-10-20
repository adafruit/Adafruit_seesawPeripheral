// Adafruit STM32 seesaw internal temperature demo.
// Select a supported STM32C011, C031, C051, or G071 generic target.
// Connect a 3.3 V USB-to-UART adapter RX to PA0 (TX), and share GND.
// Leave PA0/PA1 free of other devices. This example does not need a sensor.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_UART_DEBUG 1

#include "Adafruit_seesawPeripheral.h"

void setup() {
  Serial.setRx(PA1);
  Serial.setTx(PA0);
  Serial.begin(115200);
  delay(250);

  Serial.print("Adafruit STM32 seesaw die temperature demo\n\r");
  if (!Adafruit_seesawPeripheral_begin()) {
    Serial.print("Could not start seesaw\n\r");
    while (true) {
      delay(10);
    }
  }
  Serial.print("Chip temperature, not precision ambient temperature\n\r");
  // An I2C host can also use seesaw.getTemp() in Arduino or get_temp() in
  // CircuitPython. Temperature is automatically enabled with CONFIG_ADC.
}

void loop() {
  Adafruit_seesawPeripheral_run();

  // Keep servicing I2C between reports instead of delaying for a second.
  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    SeesawSTM32::readTemperature();
    if (g_adcStatus == 0) {
      Serial.print("Die temperature: ");
      // Whole degrees keep the demo small enough for the 32 KiB C011.
      // The I2C temperature reply still retains its 16 fractional bits.
      Serial.print(SeesawSTM32::temperature / 65536);
      Serial.print(" C\n\r");
    } else {
      Serial.print("Temperature conversion failed\n\r");
    }
  }
}
