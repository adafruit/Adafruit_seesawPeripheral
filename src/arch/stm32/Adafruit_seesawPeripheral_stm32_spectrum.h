/*!
 * @file Adafruit_seesawPeripheral_stm32_spectrum.h
 * @brief Timer-triggered ADC/DMA and fixed-point audio spectrum for STM32C0/G0.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_SPECTRUM_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_SPECTRUM_H

#include "stm32yyxx_ll_dma.h"
#include "stm32yyxx_ll_tim.h"
#include <CMSIS_DSP.h>
#include <arm_const_structs.h>

#if CONFIG_ADC
#error "STM32 spectrum capture owns ADC1; disable CONFIG_ADC"
#endif
#ifndef CONFIG_FHT_PIN
#define CONFIG_FHT_PIN 0 ///< Initial protocol GPIO index (PA0).
#endif
static_assert(CONFIG_FHT_PIN < 32 && (VALID_ADC & (1UL << CONFIG_FHT_PIN)),
              "Spectrum input must be an exposed, unreserved ADC pin");

namespace SeesawSTM32 {
const uint16_t spectrumSamples = 128; ///< Samples per transform; 64 bins.
const uint8_t spectrumGain = 8;       ///< ADC counts to signed Q15 input.
volatile uint8_t spectrumChannel = CONFIG_FHT_PIN; ///< Protocol GPIO index.
volatile uint8_t spectrumRate = 12; ///< Legacy rate code: 312500/(13+code) Hz.
volatile uint16_t spectrumADC[spectrumSamples]; ///< DMA-written 12-bit samples.
q15_t spectrumFFT[spectrumSamples * 2]; ///< Interleaved real/imaginary data.
uint8_t spectrumResults[2][spectrumSamples / 2] = {}; ///< Atomic publication.
uint8_t spectrumSnapshot[spectrumSamples / 2] = {}; ///< Latched by LOWER read.
volatile uint8_t spectrumPublished = 0; ///< Completed result buffer index.
bool spectrumRunning = false;           ///< Capture initialized successfully.
const q15_t spectrumWindow[spectrumSamples] = {
// Existing Open Music Labs Hann table; attribution retained in the include.
#include "../../dsp/hann_128.inc"
};

bool beginADC();
bool spectrumPinAvailable(uint8_t pin);

/** @brief Stop triggers before changing ADC or DMA state. */
bool stopSpectrum() {
  LL_TIM_DisableCounter(TIM3);
  if (LL_ADC_REG_IsConversionOngoing(ADC1)) {
    LL_ADC_REG_StopConversion(ADC1);
    uint32_t started = micros();
    while (LL_ADC_REG_IsConversionOngoing(ADC1)) {
      if (micros() - started > 5000) {
        spectrumRunning = false;
        return false;
      }
    }
  }
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  return true;
}

/** @brief Arm one regularly spaced frame, with no CPU sampling interrupt. */
void startSpectrum() {
  LL_DMA_ClearFlag_GI1(DMA1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, spectrumSamples);
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_ClearFlag_EOS(ADC1);
  LL_ADC_ClearFlag_OVR(ADC1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_TIM_SetCounter(TIM3, 0);
  LL_ADC_REG_StartConversion(ADC1);
  LL_TIM_EnableCounter(TIM3);
}

/** @brief Select the physical ADC channel, using the core's package pin map. */
bool selectSpectrumChannel() {
  pinMode(spectrumChannel, INPUT_ANALOG);
  uint32_t function =
      pinmap_function(digitalPinToPinName(spectrumChannel), PinMap_ADC);
  uint32_t channel = STM_PIN_CHANNEL(function);
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_REG_SetSequencerChannels(ADC1,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
  uint32_t started = micros();
  while (!LL_ADC_IsActiveFlag_CCRDY(ADC1)) {
    if (micros() - started > 5000) {
      spectrumRunning = false;
      return false;
    }
  }
  return true;
}

/** @brief Apply the legacy rate code using the timer clock, rounded to one
 * tick.
 */
void applySpectrumRate() {
  uint32_t ticks =
      (SEESAW_STM32_PCLK * (13UL + spectrumRate) + 156250) / 312500;
  LL_TIM_SetAutoReload(TIM3, ticks - 1);
  // Load the prescaler while the ADC is stopped; this emits an update trigger.
  LL_TIM_GenerateEvent_UPDATE(TIM3);
}

/** @brief Reset defaults and calibrate ADC1 before starting audio capture. */
bool beginSpectrum() {
  spectrumRunning = false;
  spectrumChannel = CONFIG_FHT_PIN;
  spectrumRate = 12;
  if (!spectrumPinAvailable(spectrumChannel)) {
    return false;
  }
  memset(spectrumResults, 0, sizeof(spectrumResults));
  memset(spectrumSnapshot, 0, sizeof(spectrumSnapshot));
  spectrumPublished = 0;
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();
  if (!beginADC()) {
    return false;
  }
  // 160.5 + 12.5 ADC cycles at 12 MHz (C0) or 16 MHz (G0) fits every rate.
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_NORMAL |
                            LL_DMA_PERIPH_NOINCREMENT |
                            LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_HALFWORD |
                            LL_DMA_MDATAALIGN_HALFWORD | LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR,
                         (uint32_t)spectrumADC,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_TIM_SetPrescaler(TIM3, 0);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  applySpectrumRate();
  if (!selectSpectrumChannel()) {
    return false;
  }
  spectrumRunning = true;
  startSpectrum();
  return true;
}

/**
 * @brief Approximate 8*log2(power), or 16*log2(amplitude), with a zero floor.
 * @param power Squared magnitude in scaled FFT units.
 * @return Log magnitude, 0..255, using linear fractional interpolation.
 */
uint8_t spectrumLog(uint32_t power) {
  if (!power) {
    return 0;
  }
  uint8_t exponent = 0;
  uint32_t normalized = power;
  while (normalized >= 2) {
    normalized /= 2;
    exponent++;
  }
  uint32_t base = 1UL << exponent;
  // Power-of-two scaling avoids a software 64-bit division on Cortex-M0+.
  uint32_t fraction;
  if (exponent >= 3) {
    fraction = (power - base) >> (exponent - 3);
  } else {
    fraction = (power - base) << (3 - exponent);
  }
  uint16_t result = exponent * 8 + fraction;
  return result > 255 ? 255 : result;
}

/** @brief Transform completed frames while the next DMA capture runs. */
void runSpectrum() {
  if (!spectrumRunning || !LL_DMA_IsActiveFlag_TC1(DMA1)) {
    return;
  }
  if (!stopSpectrum()) {
    return;
  }
  uint32_t sum = 0;
  for (uint16_t i = 0; i < spectrumSamples; i++) {
    sum += spectrumADC[i];
  }
  int32_t mean = sum / spectrumSamples;
  for (uint16_t i = 0; i < spectrumSamples; i++) {
    int32_t centered = ((int32_t)spectrumADC[i] - mean) * spectrumGain;
    spectrumFFT[i * 2] = centered * spectrumWindow[i] / 32768;
    spectrumFFT[i * 2 + 1] = 0;
  }
  startSpectrum();
  // A 128-point complex transform avoids the large shared real-FFT tables.
  // CMSIS scales Q15 forward output by 1/128; bin k is k * sample_rate / 128.
  arm_cfft_q15(&arm_cfft_sR_q15_len128, spectrumFFT, 0, 1);
  uint8_t next = 1 - spectrumPublished;
  for (uint8_t bin = 0; bin < spectrumSamples / 2; bin++) {
    int32_t real = spectrumFFT[bin * 2];
    int32_t imaginary = spectrumFFT[bin * 2 + 1];
    spectrumResults[next][bin] = spectrumLog((uint32_t)(real * real) +
                                             (uint32_t)(imaginary * imaginary));
  }
  __DMB();
  spectrumPublished = next;
}

/** @brief Return the live first lower byte before the I2C response is built. */
uint8_t firstSpectrumLower() { return spectrumResults[spectrumPublished][0]; }

/** @brief Latch all 64 bins in the I2C ISR; UPPER then uses the same frame. */
void snapshotSpectrum() {
  memcpy(spectrumSnapshot, spectrumResults[spectrumPublished],
         sizeof(spectrumSnapshot));
}
} // namespace SeesawSTM32

/** @brief Return the frame latched by the most recent LOWER read. */
const uint8_t *Adafruit_seesawPeripheral_spectrumBuffer() {
  return SeesawSTM32::spectrumSnapshot;
}

/** @brief Return the current legacy sample-rate code, 0..31. */
uint8_t Adafruit_seesawPeripheral_readSpectrumRate() {
  return SeesawSTM32::spectrumRate;
}

/** @brief Return the selected STM32 protocol GPIO index. */
uint8_t Adafruit_seesawPeripheral_readSpectrumChannel() {
  return SeesawSTM32::spectrumChannel;
}

/** @brief Change the rate between frames; clamp to the legacy 0..31 range. */
void Adafruit_seesawPeripheral_setSpectrumRate(uint8_t rate) {
  using namespace SeesawSTM32;
  if (!spectrumRunning || !stopSpectrum()) {
    return;
  }
  spectrumRate = rate > 31 ? 31 : rate;
  applySpectrumRate();
  startSpectrum();
}

/** @brief Select an exposed ADC pin; invalid or reserved pins are ignored. */
void Adafruit_seesawPeripheral_setSpectrumChannel(uint8_t pin) {
  using namespace SeesawSTM32;
  if (!spectrumPinAvailable(pin)) {
    return;
  }
  if (!spectrumRunning || !stopSpectrum()) {
    return;
  }
  pinMode(spectrumChannel, INPUT);
  spectrumChannel = pin;
  if (selectSpectrumChannel()) {
    startSpectrum();
  }
}
#endif
