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

#if CONFIG_PWM || CONFIG_PWM_16BIT
/**
 * @brief Set the hardware PWM duty cycle.
 * @param pin PWM output pin.
 * @param value 16-bit duty setting, from 0 through 65535.
 */
void Adafruit_seesawPeripheral_setPWM(uint8_t pin, uint16_t value) {
  pinMode(pin, OUTPUT);
#if CONFIG_PWM
  analogWrite(pin, value >> 8); // 8-bit PWM uses the high byte.
#elif CONFIG_PWM_16BIT
  // set duty cycle via CMPx
  uint16_t duty_cycle = map(value, 0, 0xFFFF, 0, TCA0.SINGLE.PER);
  pin -= PWM_WO_OFFSET;
  if (pin == 0) {
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;
    TCA0.SINGLE.CMP2 = duty_cycle;
  } else if (pin == 1) {
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP1EN_bm;
    TCA0.SINGLE.CMP1 = duty_cycle;
  } else if (pin == 2) {
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CMP0 = duty_cycle;
  }
#endif
}
#endif

#endif
