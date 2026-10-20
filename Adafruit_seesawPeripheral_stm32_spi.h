/*!
 * @file Adafruit_seesawPeripheral_stm32_spi.h
 * Experimental SPI-controller module, included inside SeesawSTM32.
 * Never call blocking SPI functions from a Wire interrupt.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_SPI_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_SPI_H

enum {
  spiStatusRegister =
      0x00, ///< Read flags, error, RX length, completion sequence.
  spiConfigRegister =
      0x01, ///< Mode, bit order, big-endian frequency (six bytes).
  spiTransferRegister = 0x02, ///< Flags followed by zero to 29 transmit bytes.
  spiReadRegister = 0x03,  ///< Read and consume one completed receive buffer.
  spiAbortRegister = 0x04, ///< One byte: 0 aborts; 1 also releases SPI pins.
  spiClockRegister = 0x05, ///< Read actual divided SPI clock in hertz.
};
enum {
  spiBeginFlag = 0x01,   ///< Assert CS before this chunk.
  spiEndFlag = 0x02,     ///< Deassert CS after this chunk.
  spiDiscardFlag = 0x04, ///< Discard receive data for write-only devices.
};
enum {
  spiNoError = 0, ///< No error.
  spiBadConfig =
      1,           ///< Unsupported mode, order, clock, or configuration length.
  spiBadState = 2, ///< CS sequence is invalid or the controller is disabled.
  spiQueueOverflow = 3,  ///< A command was dropped while SPI was enabled.
  spiUnreadData = 4,     ///< Previous receive data has not been consumed.
  spiTransferFailed = 5, ///< Hardware transfer failed or timed out.
  spiInvalidCommand = 6, ///< Unknown register, flags, or packet length.
  spiPinConflict = 7,    ///< SPI pins overlap another enabled peripheral.
};

/*! Release CS without changing the configured SPI pins. */
void spiRelease() {
  if (spiSelected) {
    digitalWrite(CONFIG_SPI_CS_PIN, HIGH);
    spiPort.endTransaction();
  }
  spiSelected = false;
  spiRxLength = 0;
}

/*! Disable the controller and return its four pins to unpulled inputs. */
void spiStop() {
  spiRelease();
  if (!spiConfigured)
    return;
  spiPort.end();
  spiConfigured = false;
  direction &= ~spiPinMask;
  pull &= ~spiPinMask;
#if CONFIG_PWM || CONFIG_PWM_16BIT
  pwmActive &= ~spiPinMask;
#endif
  for (uint16_t pin : spiPins)
    pinMode(pin, INPUT);
}

/*! Latch an error and release CS. @param error SPI module error code. */
void spiFail(uint8_t error) {
  spiRelease();
  spiError = error;
}

/*! Read the current SPI bridge flags without consuming receive data. */
uint8_t spiStatusFlags() {
  return (spiPending ? 1 : 0) | (spiConfigured ? 2 : 0) |
         (spiSelected ? 4 : 0) | (spiRxLength ? 8 : 0) | (spiError ? 0x80 : 0);
}
/*! Derive the SPI clock from the live hardware divider. */
uint32_t spiActualClock() {
  if (!spiConfigured)
    return 0;
  uint32_t divisor =
      ((spiPort.getHandle()->Instance->CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos) + 1;
  // begin() accepts only the target's expected PCLK. Avoid walking the RCC clock
  // tree inside the first-byte deadline; the live SPI divider is read above.
  return SEESAW_STM32_PCLK >> divisor;
}
/*! First wire byte of the actual-clock response. */
uint8_t spiFirstClockByte() { return spiActualClock() >> 24; }
/*! Peek at the first receive byte; spiRequest() consumes the full response. */
uint8_t spiFirstReceiveByte() { return spiRxLength ? spiRx[0] : 0; }

/*! Write an immediate SPI register response. @param reg SPI register. */
void spiRequest(uint8_t reg) {
  if (reg == spiStatusRegister) {
    uint8_t status[] = {spiStatusFlags(), spiError, spiRxLength, spiSequence};
    seesawWire.write(status, sizeof(status));
  } else if (reg == spiReadRegister) {
    seesawWire.write(spiRx, spiRxLength);
    spiRxLength = 0;
  } else if (reg == spiConfigRegister) {
    uint8_t config[] = {spiMode,
                        spiOrder,
                        (uint8_t)(spiFrequency >> 24),
                        (uint8_t)(spiFrequency >> 16),
                        (uint8_t)(spiFrequency >> 8),
                        (uint8_t)spiFrequency};
    seesawWire.write(config, sizeof(config));
  } else if (reg == spiClockRegister) {
    Adafruit_seesawPeripheral_write32(spiActualClock());
  }
}

/*! Process one queued SPI command in the main loop.
 * @param reg SPI register.
 * @param data Payload after the module/register prefix.
 * @param length Payload byte count.
 */
void spiProcess(uint8_t reg, const volatile uint8_t *data, uint8_t length) {
  if (reg == spiAbortRegister && length == 1 && data[0] <= 1) {
    if (data[0])
      spiStop();
    else
      spiRelease();
    spiError = spiNoError;
    spiAbortPending = false;
    return;
  }
  // Faults are sticky: queued work cannot restart a failed transaction.
  if (spiError)
    return;
  if (reg == spiConfigRegister) {
    if (length != 6 || data[0] > 3 || data[1] > 1) {
      spiFail(spiBadConfig);
      return;
    }
    uint32_t frequency = Adafruit_seesawPeripheral_read32(data + 2);
    // SPI runs from PCLK with divisors 2 through 256.
    // Reject a request below the hardware minimum instead of exceeding it.
    if (frequency < SEESAW_STM32_PCLK / 256 ||
        frequency > SEESAW_STM32_PCLK / 2 || spiSelected) {
      spiFail(spiBadConfig);
      return;
    }
    spiStop();
    uint32_t available = validGPIO();
#if CONFIG_ENCODER
    for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++)
      available &= ~((1UL << encoderPins[i][0]) | (1UL << encoderPins[i][1]));
#endif
#if CONFIG_NEOPIXEL
    if (g_neopixel_pin < SEESAW_STM32_PIN_COUNT)
      available &= ~(1UL << g_neopixel_pin);
#endif
    if ((available & spiPinMask) != spiPinMask) {
      spiFail(spiPinConflict);
      return;
    }
    // Use the core's pin maps: all three signal pins must reach the same
    // hardware SPI peripheral. CS is an ordinary software-controlled GPIO.
    void *mosi = pinmap_peripheral(digitalPinToPinName(CONFIG_SPI_MOSI_PIN),
                                   PinMap_SPI_MOSI);
    void *miso = pinmap_peripheral(digitalPinToPinName(CONFIG_SPI_MISO_PIN),
                                   PinMap_SPI_MISO);
    void *sck = pinmap_peripheral(digitalPinToPinName(CONFIG_SPI_SCK_PIN),
                                  PinMap_SPI_SCLK);
    if (mosi == NP || mosi != miso || mosi != sck) {
      spiFail(spiBadConfig);
      return;
    }
    // Stop any PWM using these exact pins before assigning the SPI function.
    direction &= ~spiPinMask;
    pull &= ~spiPinMask;
    applyGPIO(spiPinMask);
    spiMode = data[0];
    spiOrder = data[1];
    spiFrequency = frequency;
    digitalWrite(CONFIG_SPI_CS_PIN, HIGH);
    pinMode(CONFIG_SPI_CS_PIN, OUTPUT);
    const uint8_t modes[] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
    // STM32duino 3.0.0 retains its settings cache across end(). Force a
    // different setting first so an identical reconfiguration reinitializes
    // the hardware too. CS remains high during both configurations.
    uint32_t minimumFrequency = SEESAW_STM32_PCLK / 256;
    uint32_t primingFrequency = spiFrequency == minimumFrequency
                                    ? minimumFrequency * 2
                                    : minimumFrequency;
    spiPort.beginTransaction(
        SPISettings(primingFrequency, MSBFIRST, SPI_MODE0));
    spiPort.beginTransaction(SPISettings(
        spiFrequency, spiOrder ? LSBFIRST : MSBFIRST, modes[spiMode]));
    spiConfigured = true;
    return;
  }
  if (reg != spiTransferRegister || length < 1 || length > 30 ||
      (data[0] & ~7)) {
    spiFail(spiInvalidCommand);
    return;
  }
  uint8_t flags = data[0];
  if (!spiConfigured || ((flags & spiBeginFlag) ? spiSelected : !spiSelected)) {
    spiFail(spiBadState);
    return;
  }
  if (spiRxLength) {
    spiFail(spiUnreadData);
    return;
  }
  if (flags & spiBeginFlag) {
    digitalWrite(CONFIG_SPI_CS_PIN, LOW);
    spiSelected = true;
  }
  uint8_t count = length - 1;
  // Arduino transfer() discards the transfer status. Keep the HAL call here
  // to detect faults and bound a failed chunk at 50 ms; configuration uses SPI.
  // At the slowest allowed clock, 29 bytes normally take 1.24 ms.
  if (count &&
      HAL_SPI_TransmitReceive(spiPort.getHandle(), (uint8_t *)(data + 1), spiRx,
                              count, 50) != HAL_OK) {
    spiStop();
    spiFail(spiTransferFailed);
    return;
  }
  if (flags & spiEndFlag) {
    digitalWrite(CONFIG_SPI_CS_PIN, HIGH);
    spiPort.endTransaction();
    spiSelected = false;
  }
  // Publish receive data only after the complete hardware transfer.
  spiRxLength = (flags & spiDiscardFlag) ? 0 : count;
  spiSequence++;
}
#endif
