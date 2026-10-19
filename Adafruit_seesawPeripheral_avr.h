/*!
 * @file Adafruit_seesawPeripheral_avr.h
 * @brief AVR hardware access helpers for the shared seesaw protocol.
 */
#ifndef ADAFRUIT_SEESAW_PERIPHERAL_AVR_H
#define ADAFRUIT_SEESAW_PERIPHERAL_AVR_H

#if CONFIG_EEPROM
#include <EEPROM.h>

/**
 * @brief Get the hardware EEPROM capacity.
 * @return Capacity in bytes.
 */
uint16_t Adafruit_seesawPeripheral_eepromLength(void) {
  return EEPROM.length();
}

/**
 * @brief Read one byte from hardware EEPROM.
 * @param address EEPROM byte address.
 * @return Stored byte.
 */
uint8_t Adafruit_seesawPeripheral_readEEPROM(uint16_t address) {
  return EEPROM.read(address);
}

/**
 * @brief Write one byte to hardware EEPROM.
 * @param address EEPROM byte address.
 * @param value Byte to store.
 */
void Adafruit_seesawPeripheral_writeEEPROM(uint16_t address, uint8_t value) {
  EEPROM.write(address, value);
}
#endif

#if CONFIG_ADC
/**
 * @brief Read an ADC pin already checked by the command decoder.
 * @param pin Arduino pin number.
 * @return Raw ADC reading.
 */
uint16_t Adafruit_seesawPeripheral_readADC(uint8_t pin) {
  return analogRead(pin);
}
#endif

#endif
