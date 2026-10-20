/*!
 * @file Adafruit_seesawPeripheral_stm32_i2c.h
 * Bounded, allocation-free STM32 I2C target transport.
 * Fill TXDR before the first data clock; service RXNE/TXIS directly instead of
 * passing through Wire and HAL callbacks. Retain hardware stretching as a
 * safety fallback while flash writes or NeoPixel output prevent IRQ service.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_I2C_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_I2C_H

#include "PeripheralPins.h"
#include "pinmap.h"

#ifndef CONFIG_I2C_NO_STRETCH
// Bench-only override: RM0490 NOSTRETCH=1 can lose bytes whenever IRQ service
// is delayed. In particular, EEPROM writes and NeoPixel output are not safe.
#define CONFIG_I2C_NO_STRETCH 0
#endif

void seesawSTM32I2CInterrupt();
// These Cortex-M0+ targets have 16 Cortex and 32 peripheral entries. Keep every other
// handler unchanged, including SysTick, UART and timers. STM32duino's linked
// Wire code owns the flash-vector symbol, so replace only our RAM I2C1 entry.
static uint32_t seesawSTM32Vectors[48] __attribute__((aligned(256)));
static uint32_t seesawSTM32OriginalI2CVector;

/*! Minimal Wire-style target interface used by the shared seesaw handlers. */
class SeesawSTM32I2C {
public:
  /*! Record the Arduino SDA pin. */
  void setSDA(uint32_t pin) { sdaPin = digitalPinToPinName(pin); }
  /*! Record the Arduino SCL pin. */
  void setSCL(uint32_t pin) { sclPin = digitalPinToPinName(pin); }
  /*! Attach the complete-packet callback. */
  void onReceive(void (*callback)(int)) { receiveCallback = callback; }
  /*! Attach the immediate register-response callback. */
  void onRequest(void (*callback)()) { requestCallback = callback; }
  /*! Select a read source as soon as both command-header bytes arrive. */
  void onHeader(bool (*callback)(uint8_t, uint8_t)) {
    headerCallback = callback;
  }
  /*! Select a live byte source before the controller sends its read address. */
  void setFirstByte(const volatile uint8_t *source) {
    firstByteSource = source;
    firstByteCallback = nullptr;
  }
  /*! Select a short getter for a first byte that is not stored directly. */
  void setFirstByte(uint8_t (*callback)()) { firstByteCallback = callback; }

  /*! Start the dedicated I2C1 target at the board's expected clock. */
  bool begin(uint8_t address) {
    if (HAL_RCC_GetPCLK1Freq() != SEESAW_STM32_PCLK ||
        pinmap_peripheral(sdaPin, PinMap_I2C_SDA) != I2C1 ||
        pinmap_peripheral(sclPin, PinMap_I2C_SCL) != I2C1)
      return false;
    NVIC_DisableIRQ(I2C1_IRQn);
    __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_PCLK1);
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    pinmap_pinout(sdaPin, PinMap_I2C_SDA);
    pinmap_pinout(sclPin, PinMap_I2C_SCL);
    // SCLH/L do not generate clocks in target mode; this is never a controller.
#if SEESAW_STM32_PCLK == 64000000UL
    // At 64 MHz: PRESC=1, SCLDEL=9 (312.5 ns), SDADEL=3 (93.75 ns).
    I2C1->TIMINGR =
        (1UL << I2C_TIMINGR_PRESC_Pos) | (9UL << I2C_TIMINGR_SCLDEL_Pos) |
        (3UL << I2C_TIMINGR_SDADEL_Pos) | (39UL << I2C_TIMINGR_SCLH_Pos) | 39UL;
#else
    // At 48 MHz: PRESC=0, SCLDEL=14 (312.5 ns), SDADEL=4 (83.3 ns).
    I2C1->TIMINGR = (14UL << I2C_TIMINGR_SCLDEL_Pos) |
                    (4UL << I2C_TIMINGR_SDADEL_Pos) |
                    (59UL << I2C_TIMINGR_SCLH_Pos) | 59UL;
#endif
    I2C1->OAR1 = I2C_OAR1_OA1EN | ((uint32_t)address << 1);
    I2C1->OAR2 = 0;
    I2C1->CR2 = 0;
    rxLength = rxIndex = txLength = txIndex = 0;
    receiving = transmitting = false;
    overruns = busErrors = firstByteMismatches = 0;
    I2C1->CR1 = I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_STOPIE |
                I2C_CR1_NACKIE | I2C_CR1_ERRIE;
#if CONFIG_I2C_NO_STRETCH
    I2C1->CR1 |= I2C_CR1_NOSTRETCH;
#endif
    I2C1->CR1 |= I2C_CR1_PE;
    // Highest priority: a 400 kHz byte arrives every 22.5 microseconds.
    NVIC_SetPriority(I2C1_IRQn, 0);
    NVIC_ClearPendingIRQ(I2C1_IRQn);
    uint32_t savedMask = __get_PRIMASK();
    __disable_irq();
    if (SCB->VTOR != (uintptr_t)seesawSTM32Vectors) {
      memcpy(seesawSTM32Vectors, (const void *)SCB->VTOR,
             sizeof(seesawSTM32Vectors));
      seesawSTM32OriginalI2CVector = seesawSTM32Vectors[16 + I2C1_IRQn];
    }
    seesawSTM32Vectors[16 + I2C1_IRQn] = (uintptr_t)seesawSTM32I2CInterrupt;
    SCB->VTOR = (uintptr_t)seesawSTM32Vectors;
    __DSB();
    __ISB();
    __set_PRIMASK(savedMask);
    NVIC_EnableIRQ(I2C1_IRQn);
    return true;
  }

  /*! Stop this target before changing its address. */
  void end() {
    NVIC_DisableIRQ(I2C1_IRQn);
    I2C1->CR1 = 0;
    NVIC_ClearPendingIRQ(I2C1_IRQn);
    if (SCB->VTOR == (uintptr_t)seesawSTM32Vectors)
      seesawSTM32Vectors[16 + I2C1_IRQn] = seesawSTM32OriginalI2CVector;
  }

  /*! Bytes remaining in the complete received packet. */
  int available() const {
    uint8_t stored = rxLength > sizeof(rxBuffer) ? sizeof(rxBuffer) : rxLength;
    return stored - rxIndex;
  }
  /*! Inspect the next byte without consuming it. */
  int peek() const { return available() ? rxBuffer[rxIndex] : -1; }
  /*! Consume the next byte. */
  int read() { return available() ? rxBuffer[rxIndex++] : -1; }
  /*! Append one byte to the pending reply. Called only from requestCallback. */
  __attribute__((always_inline)) inline size_t write(uint8_t value) {
    if (txLength == sizeof(txBuffer))
      return 0;
    txBuffer[txLength++] = value;
    return 1;
  }
  /*! Append a bounded reply block. */
  size_t write(const uint8_t *data, size_t length) {
    size_t room = sizeof(txBuffer) - txLength;
    if (length > room)
      length = room;
    memcpy(txBuffer + txLength, data, length);
    txLength += length;
    return length;
  }

  /*! Hardware data overruns/underruns, retained for bench verification. */
  volatile uint32_t overruns = 0;
  /*! Bus/arbitration errors, retained for bench verification. */
  volatile uint32_t busErrors = 0;
  /*! The fast byte must match the same read's ordinary protocol response. */
  volatile uint32_t firstByteMismatches = 0;

  /*! Service the combined I2C interrupt. No waits or allocation. */
  __attribute__((always_inline)) inline void interrupt() {
    uint32_t status = I2C1->ISR;
    // A read address has less than one clock to load its first data byte.
    // Handle the normal case before checking unrelated error/STOP flags.
    if ((status & (I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_RXNE | I2C_ISR_STOPF |
                   I2C_ISR_NACKF)) == (I2C_ISR_ADDR | I2C_ISR_DIR)) {
      if (receiving && (rxLength != 2 || !deferReceive))
        completeReceive();
      startTransmit();
      return;
    }
    if ((status & (I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_RXNE | I2C_ISR_STOPF |
                   I2C_ISR_NACKF)) == I2C_ISR_ADDR) {
      // A write address needs no reply. Release it before packet housekeeping;
      // there is a full data byte's time before the first RXNE deadline.
      I2C1->ICR = I2C_ICR_ADDRCF;
      completeReceive();
      transmitting = false;
      I2C1->CR1 &= ~I2C_CR1_TXIE;
      I2C1->ISR = I2C_ISR_TXE;
      rxLength = rxIndex = 0;
      deferReceive = false;
      receiving = true;
      return;
    }
    if (status & I2C_ISR_OVR) {
      overruns++;
      I2C1->ICR = I2C_ICR_OVRCF;
    }
    if (status & (I2C_ISR_BERR | I2C_ISR_ARLO)) {
      busErrors++;
      I2C1->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    }
    // The final RXNE may coincide with STOP or a repeated START.
    if (status & I2C_ISR_RXNE) {
      uint8_t value = I2C1->RXDR;
      if (rxLength < sizeof(rxBuffer))
        rxBuffer[rxLength++] = value;
      else
        rxLength =
            sizeof(rxBuffer) + 1; // Report an oversized packet, not a prefix.
      if (rxLength == 2 && headerCallback)
        deferReceive = headerCallback(rxBuffer[0], rxBuffer[1]);
    }
    if (status & I2C_ISR_NACKF) {
      transmitting = false;
      I2C1->CR1 &= ~I2C_CR1_TXIE;
      I2C1->ICR = I2C_ICR_NACKCF;
    }
    if (status & I2C_ISR_STOPF) {
      I2C1->ICR = I2C_ICR_STOPCF;
      completeReceive();
      transmitting = false;
      I2C1->CR1 &= ~I2C_CR1_TXIE;
      I2C1->ISR = I2C_ISR_TXE; // Flush any unclocked reply byte.
    }
    if (status & I2C_ISR_ADDR) {
      if (status & I2C_ISR_DIR) {
        if (receiving && (rxLength != 2 || !deferReceive))
          completeReceive();
        startTransmit();
      } else {
        completeReceive();
        transmitting = false;
        I2C1->CR1 &= ~I2C_CR1_TXIE;
        I2C1->ISR = I2C_ISR_TXE;
        rxLength = rxIndex = 0;
        deferReceive = false;
        receiving = true;
      }
      I2C1->ICR = I2C_ICR_ADDRCF;
    } else if ((status & I2C_ISR_TXIS) && transmitting) {
      I2C1->TXDR = txIndex < txLength ? txBuffer[txIndex++] : 0;
    }
  }

private:
  /*! Start the reply before doing nonessential interrupt bookkeeping. */
  __attribute__((always_inline)) inline void startTransmit() {
    // Source selection was done at the preceding write. Read its LIVE value
    // now, while main-loop writers cannot run, then build the same snapshot.
    uint8_t first = firstByteCallback ? firstByteCallback() : *firstByteSource;
    I2C1->TXDR = first;
    I2C1->ICR = I2C_ICR_ADDRCF;
    // A repeated START need not wait for queue bookkeeping: the two-byte
    // header already selected the source on RXNE. The first byte is in flight.
    if (receiving)
      completeReceive();
    txLength = 0;
    txIndex = 1;
    transmitting = true;
    if (requestCallback)
      requestCallback();
    if (first != (txLength ? txBuffer[0] : 0))
      firstByteMismatches++;
    I2C1->CR1 |= I2C_CR1_TXIE;
  }

  PinName sdaPin = PB_7, sclPin = PB_6; ///< Physical I2C pins.
  uint8_t rxBuffer[32], txBuffer[32];   ///< Bounded classic seesaw packets.
  uint8_t rxLength = 0, rxIndex = 0;    ///< Complete-packet read cursor.
  uint8_t txLength = 0, txIndex = 0;    ///< Current reply cursor.
  bool receiving = false, transmitting = false; ///< Active transfer direction.
  bool deferReceive =
      false;              ///< Two-byte header cannot be rejected by the queue.
  const uint8_t zero = 0; ///< Safe response before any register is selected.
  const volatile uint8_t *firstByteSource = &zero; ///< Live first-byte source.
  uint8_t (*firstByteCallback)() =
      nullptr;                            ///< Optional short first-byte getter.
  void (*receiveCallback)(int) = nullptr; ///< Packet decoder entry point.
  void (*requestCallback)() = nullptr;    ///< Register response entry point.
  bool (*headerCallback)(uint8_t,
                         uint8_t) = nullptr; ///< Early register selection.

  /*! Deliver a complete write once, at STOP or a repeated START. */
  void completeReceive() {
    if (receiving && rxLength && receiveCallback) {
      rxIndex = 0;
      receiveCallback(rxLength);
    }
    receiving = false;
  }
};

SeesawSTM32I2C seesawWire; ///< Dedicated target; does not own global Wire.

/** @brief Write one byte through the STM32 target transport. */
__attribute__((always_inline)) inline void
Adafruit_seesawPeripheral_write8(uint8_t value) {
  seesawWire.write(value);
}

/** @brief Write a bounded byte buffer through the STM32 target transport. */
__attribute__((always_inline)) inline void
Adafruit_seesawPeripheral_writeBuffer(const uint8_t *data, size_t length) {
  seesawWire.write(data, length);
}

/*! This transport owns I2C1, independently of STM32duino's Wire HAL. */
// The standard STM32 linker/startup copies .RamFunc with .data. Avoid flash
// wait states on the address/first-byte path; callbacks still live in flash.
__attribute__((section(".RamFunc"), noinline)) void seesawSTM32I2CInterrupt() {
  seesawWire.interrupt();
}

#endif
