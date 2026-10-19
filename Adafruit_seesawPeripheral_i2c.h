/*!
 * @file Adafruit_seesawPeripheral_i2c.h
 * @brief I2C callbacks for the shared seesaw protocol.
 */
#ifndef ADAFRUIT_SEESAW_PERIPHERAL_I2C_H
#define ADAFRUIT_SEESAW_PERIPHERAL_I2C_H

/***************************** data write */

void receiveEvent(int howMany) {
  SEESAW_DEBUG(F("Received "));
  SEESAW_DEBUG(howMany);
  SEESAW_DEBUG(F(" bytes:"));
  // Reject invalid packets before changing the selected module/register.
  if (howMany < 2 || (uint32_t)howMany > sizeof(i2c_buffer)) {
    while (Wire.available()) {
      Wire.read();
    }
    SEESAW_DEBUGLN();
    return;
  }
  for (uint8_t i = 0; i < howMany; i++) {
    i2c_buffer[i] = Wire.read();
    //SEESAW_DEBUG(F("0x"));
    //SEESAW_DEBUG(i2c_buffer[i], HEX);
    //SEESAW_DEBUG(F(" "));
  }
  //SEESAW_DEBUG("\n");

  // Each command validates its length before accessing payload bytes.
  Adafruit_seesawPeripheral_processCommand(i2c_buffer, howMany);
}

/***************************** data read */
void requestEvent(void) {
  // SEESAW_DEBUGLN(F("Requesting data"));
  uint8_t base_cmd = i2c_buffer[0];
  uint8_t module_cmd = i2c_buffer[1];

  Adafruit_seesawPeripheral_processRequest(base_cmd, module_cmd);
}

#endif
